#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "nvs_flash.h"
#include "esp_random.h"
#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "max30100.h"
#include "esp_task_wdt.h"
#include "driver/uart.h"

// Configuration ADC et MQ135
#define MQ135_ADC_CHANNEL ADC_CHANNEL_0
#define ADC_RESOLUTION 4095.0
#define V_REF 3.3
#define RL 10000
#define A 116.6020682
#define B -2.769034857
#define PREHEAT_TIME_MS 30000  // 30 secondes de préchauffage
#define CALIBRATION_SAMPLE_TIMES 50
#define CALIBRATION_SAMPLE_INTERVAL 500  // 500ms entre chaque échantillon
#define GPS_UART_NUM UART_NUM_2
#define GPS_TX_PIN GPIO_NUM_17
#define GPS_RX_PIN GPIO_NUM_16
#define GPS_BUFFER_SIZE 1024
// Configuration I2C pour MPU6050
#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000
#define MPU6050_ADDR 0x68
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_ACCEL_XOUT_H 0x3B
#define ACCEL_SCALE_FACTOR 16384.0

// Configuration I2C pour MAX30100
#define MAX30100_SDA 26
#define MAX30100_SCL 25
#define MAX30100_I2C_PORT I2C_NUM_1

// Configuration GPIO pour DHT22
#define DHT_GPIO_PIN GPIO_NUM_4

// Structure pour données DHT22
typedef struct {
    float temperature;
    float humidity;
} dht_data_t;

// Variables globales
static adc_oneshot_unit_handle_t adc1_handle;
static float r0 = 0;  // Valeur de référence pour le MQ135 (résistance après pré-chauffage)
static float temp = 0;
static float humi = 0;
static float current_bpm = 0;  // Variable globale pour stocker le BPM
static float current_spo2 = 0; // Variable globale pour stocker le SpO2
static max30100_config_t max30100 = {};
static const char *TAG = "BLE-Server";
static uint8_t ble_addr_type;
static uint16_t notify_handle;

static float current_latitude = 0.0f;  // Variable globale pour la latitude
static float current_longitude = 0.0f; // Variable globale pour la longitude
static float current_speed = 0.0f;     // Variable globale pour la vitesse en km/h

//variable pour les données envoyés
static float last_valid_denivele = 0;
static float last_valid_bpm = 0;
static float last_valid_spo2 = 0;
static float last_valid_co2 = 0;
static float last_valid_temp = 0;
static float last_valid_hum = 0;
// Flags pour la gestion des connexions I2C
static bool mpu6050_connected = false;
static bool max30100_connected = false;



// Prototypes des fonctions
void get_bpm(void* param);
void json_send_task(void *param);
static void try_connect_mpu6050(void* pvParameters);
static void try_connect_max30100(void* pvParameters);

static esp_err_t i2c_master_init_mpu6050(void);
static esp_err_t i2c_master_init_max30100(void);
static esp_err_t mpu6050_init(void);
static void dht_init(void);
static int dht_read_bit(void);
static int dht_read_byte(void);
static int dht_read_data(dht_data_t *data);
void ble_app_advertise(void);
void generate_json(char *buffer, size_t size);
float calibrate_mq135(void);
float calculate_ppm(float rs, float r0);
float calculate_rs(int adc_value);
int read_adc(void);



void init_adc(void) {
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN_DB_12,  //config à 3.3V
        .bitwidth = ADC_BITWIDTH_12, //rep sur 12 bits 
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, MQ135_ADC_CHANNEL, &config));
    
    r0 = calibrate_mq135();
}

// Lecture de l'ADC
int read_adc(void) {
    int adc_raw;
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, MQ135_ADC_CHANNEL, &adc_raw));
    return adc_raw;
}

// Calibration du MQ135
float calibrate_mq135() {
    float avg_rs = 0;
    
    ESP_LOGI(TAG, "Préchauffage du MQ135...");
    vTaskDelay(pdMS_TO_TICKS(PREHEAT_TIME_MS));
    
    ESP_LOGI(TAG, "Début de la calibration MQ135...");
    for(int i = 0; i < CALIBRATION_SAMPLE_TIMES; i++) {
        int adc_value = read_adc();
        float rs = calculate_rs(adc_value);
        if(rs > 0) {
            avg_rs += rs;
        }
        vTaskDelay(pdMS_TO_TICKS(CALIBRATION_SAMPLE_INTERVAL));
    }
    
    avg_rs = avg_rs / CALIBRATION_SAMPLE_TIMES;
    float r0_value = avg_rs / 3.6;
    ESP_LOGI(TAG, "Calibration MQ135 terminée, R0 = %.2f", r0_value);
    return r0_value;
}

// Calcul de Rs pour le MQ135
float calculate_rs(int adc_value) {
    float v_out = (adc_value / (float)ADC_RESOLUTION) * V_REF;
    if (v_out == 0) return -1;
    float rs = RL * (V_REF - v_out) / v_out;
    return rs;
}

// Calcul de la concentration en PPM
float calculate_ppm(float rs, float r0) {
    float ratio = rs / r0;
    float ppm = A * pow(ratio, B);
    return ppm;
}

//Fonction de conversion NMEA (Les trams qui arrivent et sont changées en coordonnées)
static float nmea_to_decimal(const char* nmea_pos, char dir) {
    if (!nmea_pos || strlen(nmea_pos) < 6) {
        return 0.0f;
    }

    char deg_str[4] = {0};
    char min_str[10] = {0};
    
    if (strlen(nmea_pos) == 9 || strlen(nmea_pos) == 10) {
        strncpy(deg_str, nmea_pos, 2);
        strcpy(min_str, nmea_pos + 2);
    } else {
        strncpy(deg_str, nmea_pos, 3);
        strcpy(min_str, nmea_pos + 3);
    }
    
    float deg = atof(deg_str);
    float min = atof(min_str);
    float decimal = deg + (min / 60.0f);
    
    if (dir == 'S' || dir == 'W') {
        decimal = -decimal;
    }
    
    return decimal;
}

//Fonction d'initialisation du GPS
static void gps_init(void) {
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    ESP_ERROR_CHECK(uart_driver_install(GPS_UART_NUM, GPS_BUFFER_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(GPS_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(GPS_UART_NUM, GPS_TX_PIN, GPS_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

//Tâche de lecture GPS
static void gps_read_task(void *param) {
    uint8_t data[GPS_BUFFER_SIZE];
    char line[128];
    int line_pos = 0;

    while (1) {
        int length = uart_read_bytes(GPS_UART_NUM, data, GPS_BUFFER_SIZE - 1, 500 / portTICK_PERIOD_MS);
        
        if (length > 0) {
            data[length] = 0;
            
            for(int i = 0; i < length; i++) {
                if(data[i] == '\n') {
                    line[line_pos] = 0;
                    
                    if(strstr(line, "GNRMC") && strstr(line, ",A,")) {
                        char *saveptr;
                        
                        strtok_r(line, ",", &saveptr);     // $GNRMC
                        strtok_r(NULL, ",", &saveptr);     // Temps
                        strtok_r(NULL, ",", &saveptr);     // Status
                        
                        char *lat = strtok_r(NULL, ",", &saveptr);
                        char *lat_dir = strtok_r(NULL, ",", &saveptr);
                        char *lon = strtok_r(NULL, ",", &saveptr);
                        char *lon_dir = strtok_r(NULL, ",", &saveptr);
                        char *speed = strtok_r(NULL, ",", &saveptr);
                        
                        if(lat && lat_dir && lon && lon_dir && speed) {
                            current_latitude = nmea_to_decimal(lat, *lat_dir);
                            current_longitude = nmea_to_decimal(lon, *lon_dir);
                            current_speed = atof(speed) * 1.852f; // Conversion en km/h de la vitesse
                            
                            ESP_LOGI(TAG, "GPS: %.6f, %.6f, %.1f km/h", 
                                   current_latitude, current_longitude, current_speed);
                        }
                    }
                    
                    line_pos = 0;
                } else if(data[i] != '\r' && line_pos < sizeof(line)-1) {
                    line[line_pos++] = data[i];
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
// Fonctions MPU6050 et config de la liaison I2C
static esp_err_t i2c_write_byte(uint8_t dev_addr, uint8_t reg_addr, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t i2c_read_bytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t mpu6050_init(void) {
    return i2c_write_byte(MPU6050_ADDR, MPU6050_PWR_MGMT_1, 0x00);
}

static void mpu6050_read_angle(float *denivele) {
    if (!mpu6050_connected) {
        *denivele = 0;
        return;
    }

    uint8_t data[6];
    esp_err_t ret = i2c_read_bytes(MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, data, 6);
    if (ret != ESP_OK) {
        mpu6050_connected = false;
        *denivele = 0;
        return;
    }

    int16_t raw_x = (int16_t)((data[0] << 8) | data[1]);
    int16_t raw_z = (int16_t)((data[4] << 8) | data[5]);

    float accel_x = raw_x / ACCEL_SCALE_FACTOR;
    float accel_z = raw_z / ACCEL_SCALE_FACTOR;

    // Calcul de l'angle en degrés
    float angle = atan2(accel_x, accel_z) * (180.0 / M_PI);
    
    // Conversion en pourcentage de pente (0-100%)
    // Avec une référence à 100° et max à 190° (90° de plus)
    *denivele = ((angle - 100.0) / 90.0) * 100.0;
    
    // Limiter entre 0 et 100%
    if (*denivele < 0) *denivele = 0;
    if (*denivele > 100) *denivele = 100;
}

// Fonctions pour DHT22
static void dht_init() {
    gpio_set_direction(DHT_GPIO_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(DHT_GPIO_PIN, 1);
}

static int dht_read_bit() {
    int timeout = 0;
    while (gpio_get_level(DHT_GPIO_PIN) == 0) {
        if (timeout++ > 100) return -1;
        esp_rom_delay_us(1);
    }
    uint32_t start_time = esp_timer_get_time();
    timeout = 0;
    while (gpio_get_level(DHT_GPIO_PIN) == 1) {
        if (timeout++ > 100) return -1;
        esp_rom_delay_us(1);
    }
    uint32_t end_time = esp_timer_get_time();
    return (end_time - start_time) > 40;
}

static int dht_read_byte() {
    int result = 0;
    for (int i = 0; i < 8; i++) {
        int bit = dht_read_bit();
        if (bit < 0) return -1;  // Erreur de lecture
        result = (result << 1) | bit;
    }
    return result;
}

static int dht_read_data(dht_data_t *data) {
    uint8_t bits[5] = {0};
    int timeout = 0;
    
    gpio_set_direction(DHT_GPIO_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(DHT_GPIO_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(DHT_GPIO_PIN, 1);
    esp_rom_delay_us(40);
    gpio_set_direction(DHT_GPIO_PIN, GPIO_MODE_INPUT);

    while (gpio_get_level(DHT_GPIO_PIN) == 1) {
        if (timeout++ > 100) return ESP_FAIL;
        esp_rom_delay_us(1);
    }

    for (int i = 0; i < 5; i++) {
        int byte_read = dht_read_byte();
        if (byte_read < 0) return ESP_FAIL;
        bits[i] = (uint8_t)byte_read;
    }

    if (bits[4] != ((bits[0] + bits[1] + bits[2] + bits[3]) & 0xFF)) {
        return ESP_FAIL;
    }

    data->humidity = ((bits[0] << 8) | bits[1]) * 0.1f;
    data->temperature = (((bits[2] & 0x7F) << 8) | bits[3]) * 0.1f;
    if (bits[2] & 0x80) {
        data->temperature = -data->temperature;
    }

    return ESP_OK;
}
// Fonction de tentative de connexion I2C pour MPU6050
static void try_connect_mpu6050(void* pvParameters) {
    while (!mpu6050_connected) {
        esp_err_t ret = i2c_master_init_mpu6050();
        if (ret == ESP_OK) {
            ret = mpu6050_init();
            if (ret == ESP_OK) {
                mpu6050_connected = true;
                ESP_LOGI(TAG, "MPU6050 connecté avec succès");
                break;
            }
        }
        ESP_LOGW(TAG, "Échec connexion MPU6050, nouvelle tentative dans 5s");
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
    vTaskDelete(NULL);
}

// Fonction de tentative de connexion I2C pour MAX30100
static void try_connect_max30100(void* pvParameters) {
    while (!max30100_connected) {
        esp_err_t ret = i2c_master_init_max30100();
        if (ret == ESP_OK) {
            max30100.acceptable_intense_diff = MAX30100_DEFAULT_ACCEPTABLE_INTENSITY_DIFF;
            max30100.red_current_adj_ms = MAX30100_DEFAULT_RED_LED_CURRENT_ADJUSTMENT_MS;
            max30100.reset_spo2_pulse_n = MAX30100_DEFAULT_RESET_SPO2_EVERY_N_PULSES;
            
            ret = max30100_init(&max30100, MAX30100_I2C_PORT,
                               MAX30100_MODE_SPO2_HR,
                               MAX30100_SAMPLING_RATE_100HZ,
                               MAX30100_PULSE_WIDTH_1600US_ADC_16,
                               MAX30100_LED_CURRENT_50MA,
                               MAX30100_LED_CURRENT_27_1MA,
                               15,
                               10,
                               true,
                               true);
            
            if (ret == ESP_OK) {
                max30100_connected = true;
                ESP_LOGI(TAG, "MAX30100 connecté avec succès");
                break;
            }
        }
        ESP_LOGW(TAG, "Échec connexion MAX30100, nouvelle tentative dans 5s");
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
    vTaskDelete(NULL);
}

// Task pour la lecture du MAX30100
void get_bpm(void* param) {
    printf("MAX30100 Task Started\n");
    max30100_data_t result = {};
    
    while(true) {
        if (max30100_connected) {
            esp_err_t ret = max30100_update(&max30100, &result);
            if (ret == ESP_OK && result.pulse_detected) {
                ESP_LOGI(TAG, "BEAT Detected - BPM: %f | SpO2: %f%%", result.heart_bpm, result.spO2);
                current_bpm = result.heart_bpm;
                current_spo2 = result.spO2;
            } else if (ret != ESP_OK) {
                max30100_connected = false;
                xTaskCreate(try_connect_max30100, "connect_max30100", 4096, NULL, 5, NULL);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// i2c_master_init_mpu6050 pour l'initialisation de l'I2C du MPU6050
static esp_err_t i2c_master_init_mpu6050(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

// i2c_master_init_max30100 pour l'initialisation de l'I2C du MAX30100
static esp_err_t i2c_master_init_max30100(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = MAX30100_SDA,
        .scl_io_num = MAX30100_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
    };
    i2c_param_config(MAX30100_I2C_PORT, &conf);
    return i2c_driver_install(MAX30100_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
}
void generate_json(char *buffer, size_t size) {
    // Lecture de la dénivelée
    float current_denivele;
    mpu6050_read_angle(&current_denivele);
    if (current_denivele > 0) {
        last_valid_denivele = current_denivele;
    }

    // Lecture MQ135 et conversion directe en pourcentage 0-10
    float adc_value = read_adc();
    float pollution_percent = (adc_value/10000)*100;


    // Lecture DHT22
    dht_data_t dht_data = {0};
    if (dht_read_data(&dht_data) == ESP_OK) {
        if (dht_data.temperature != 0) {
            last_valid_temp = dht_data.temperature;
            temp = dht_data.temperature;
        }
        if (dht_data.humidity != 0) {
            last_valid_hum = dht_data.humidity;
            humi = dht_data.humidity;
        }
    }
    
    // Mise à jour des valeurs du MAX30100 si valides et ajustement du BPM
    if (current_bpm > 0) {
        if (current_bpm > 100) {
            last_valid_bpm = current_bpm - 100;
        } else {
            last_valid_bpm = current_bpm;
        }
    }
    if (current_spo2 > 0) {
        last_valid_spo2 = current_spo2;
    }
    
    // Log des valeurs pour debug
    ESP_LOGI(TAG, "Denivele: %.1f%%, BPM: %.1f, SpO2: %.1f%%, Pollution: %.1f%%, Temp: %.1f°C, Hum: %.1f%%",
             last_valid_denivele, last_valid_bpm, last_valid_spo2, 
             pollution_percent, last_valid_temp, last_valid_hum);

      snprintf(buffer, size,
             "{\n"
             "  \"bpm\": %.1f,\n"
             "  \"denivele\": %.1f,\n"
             "  \"temperature\": %.1f,\n"
             "  \"humidity\": %.1f,\n"
             "  \"pollution\": %.1f,\n"
             "  \"spo2\": %.1f,\n"
             "  \"latitude\": %.6f,\n"
             "  \"longitude\": %.6f,\n"
             "  \"speed\": %.1f\n"
             "}",
             last_valid_bpm, last_valid_denivele, 
             last_valid_temp, last_valid_hum, 
             pollution_percent, last_valid_spo2,
             current_latitude, current_longitude, current_speed);
}
//Commande si on veut tester juste avec l'app
/*snprintf(buffer, size,
             "{\"vitesse\": %.1f, \"bpm\": %.1f, \"altitude\": %.1f}"
                ,current_speed, last_valid_bpm, last_valid_denivele);*/
// Task pour envoyer les données JSON périodiquement
void json_send_task(void *param) {
    char json_buffer[256];
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t frequency = pdMS_TO_TICKS(1000); // 1 Hz
    
    // S'enregistrer auprès du watchdog (superviseur qui gère les exécutions)
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
    
    while (1) {
        // Reset watchdog timer
        ESP_ERROR_CHECK(esp_task_wdt_reset());
        
        generate_json(json_buffer, sizeof(json_buffer));
        struct os_mbuf *om = ble_hs_mbuf_from_flat(json_buffer, strlen(json_buffer));
        if (om != NULL) {
            int rc = ble_gattc_notify_custom(0, notify_handle, om);
            if (rc != 0) {
                ESP_LOGW(TAG, "Failed to send notification, rc: %d", rc);
            }
        }
        
        vTaskDelayUntil(&last_wake_time, frequency);
    }
}

// BLE Write characteristic
static int device_write(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    return 0;
}

// BLE Read characteristic
static int device_read(uint16_t con_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    char json_buffer[256];
    generate_json(json_buffer, sizeof(json_buffer));
    os_mbuf_append(ctxt->om, json_buffer, strlen(json_buffer));
    return 0;
}

// BLE GATT Services
static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(0x180),
        .characteristics = (struct ble_gatt_chr_def[]){
            {
                .uuid = BLE_UUID16_DECLARE(0x2222),
                .flags = BLE_GATT_CHR_F_NOTIFY,
                .access_cb = device_read,
                .val_handle = &notify_handle
            },
            {
                .uuid = BLE_UUID16_DECLARE(0xDEAD),
                .flags = BLE_GATT_CHR_F_WRITE,
                .access_cb = device_write
            },
            {0}
        }
    },
    {0}
};

// BLE GAP events
static int ble_gap_event(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            ESP_LOGI(TAG, "BLE GAP EVENT CONNECT %s", event->connect.status == 0 ? "OK!" : "FAILED!");
            if (event->connect.status != 0) {
                ble_app_advertise();
            }
            break;
        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI(TAG, "BLE GAP EVENT DISCONNECTED");
            ble_app_advertise();
            break;
        case BLE_GAP_EVENT_ADV_COMPLETE:
            ESP_LOGI(TAG, "BLE GAP EVENT ADV COMPLETE");
            ble_app_advertise();
            break;
        default:
            break;
    }
    return 0;
}

// BLE Advertising
void ble_app_advertise(void) {
    struct ble_hs_adv_fields fields;
    memset(&fields, 0, sizeof(fields));
    fields.name = (uint8_t *)ble_svc_gap_device_name();
    fields.name_len = strlen(ble_svc_gap_device_name());
    fields.name_is_complete = 1;
    ble_gap_adv_set_fields(&fields);

    struct ble_gap_adv_params adv_params = {0};
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    ble_gap_adv_start(ble_addr_type, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event, NULL);
}

void ble_app_on_sync(void) {
    ble_hs_id_infer_auto(0, &ble_addr_type);
    ble_app_advertise();
}

void host_task(void *param) {
    nimble_port_run();
}

void app_main() {
    // Initialisation de la mémoire NVS
    ESP_ERROR_CHECK(nvs_flash_init());

    // Désactiver le watchdog timer s'il est déjà initialisé
    esp_task_wdt_deinit();
    gps_init();

    // Configuration du watchdog timer
    esp_task_wdt_config_t twdt_config = {
        .timeout_ms = 10000,                // 10 secondes
        .idle_core_mask = (1 << 0),         // Surveillance du core 0
        .trigger_panic = false              // Pas de panic en cas de timeout
    };
    ESP_ERROR_CHECK(esp_task_wdt_init(&twdt_config));

    // Création des tâches de connexion I2C
    xTaskCreate(try_connect_mpu6050, "connect_mpu6050", 4096, NULL, 5, NULL);
    xTaskCreate(try_connect_max30100, "connect_max30100", 4096, NULL, 5, NULL);
    
    // Initialisation de l'ADC pour le MQ135
    init_adc();

    // Initialisation du DHT22
    dht_init();

    // Initialisation du BLE
    nimble_port_init();
    ble_svc_gap_device_name_set("BLE-Random");
    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_gatts_count_cfg(gatt_svcs);
    ble_gatts_add_svcs(gatt_svcs);
    ble_hs_cfg.sync_cb = ble_app_on_sync;

    nimble_port_freertos_init(host_task);

    // Création des tâches principales sur des cores différents
    xTaskCreatePinnedToCore(
        get_bpm,            // Tâche de lecture MAX30100
        "Get BPM",
        8192,
        NULL,
        10,                 // Priorité haute
        NULL,
        1                   // Core 1
    );
       xTaskCreatePinnedToCore(
        gps_read_task,      // Tâche de lecture GPS
        "gps_read_task",
        4096,
        NULL,
        5,                  // Priorité moyenne
        NULL,
        1                   // Core 1
    );

    xTaskCreatePinnedToCore(
        json_send_task,     // Tâche d'envoi BLE
        "json_send_task",
        4096,
        NULL,
        5,                  // Priorité moyenne
        NULL,
        0                   // Core 0
    );
  
}