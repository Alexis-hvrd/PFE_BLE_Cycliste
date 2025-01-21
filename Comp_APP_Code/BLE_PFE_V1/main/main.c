#include <stdio.h>
#include "driver/i2c.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#include "max30100.h"
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "nvs_flash.h"


#define I2C_SDA 26
#define I2C_SCL 25
#define I2C_FRQ 100000
#define I2C_PORT I2C_NUM_0

max30100_config_t max30100 = {};
char *TAG = "BLE-Server";
uint8_t ble_addr_type;
static uint16_t notify_handle; // Handle pour les notifications

// Déclaration de la fonction `ble_app_advertise` ici
void ble_app_advertise(void);

// Structure pour stocker les données JSON
typedef struct {
    int vitesse;
    int bpm;
    int altitude;
} SensorData;

// Données JSON mises à jour en boucle
static SensorData sensor_data = {0, 0, 0};

esp_err_t i2c_master_init(i2c_port_t i2c_port){
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_SDA;
    conf.scl_io_num = I2C_SCL;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_FRQ;
    i2c_param_config(i2c_port, &conf);
    return i2c_driver_install(i2c_port, I2C_MODE_MASTER, 0, 0, 0);
}

void get_bpm(void* param) {
    printf("MAX30100 Test\n");
    max30100_data_t result = {};
    while(true) {
        //Update sensor, saving to "result"
        ESP_ERROR_CHECK(max30100_update(&max30100, &result));
        if(result.pulse_detected) {
            printf("BEAT\n");
            printf("BPM: %f | SpO2: %f%%\n", result.heart_bpm, result.spO2);
        }
        //Update rate: 100Hz
        vTaskDelay(10/portTICK_PERIOD_MS);
    }
}

// Génère un nombre aléatoire entre 1 et 10
int generate_random_value() {
    return (rand() % 10) + 1;
}

// Tâche pour mettre à jour les données JSON toutes les secondes
void update_sensor_data_task(void *param) {
    while (1) {
        max30100_data_t result = {};
        ESP_ERROR_CHECK(max30100_update(&max30100, &result));

        sensor_data.vitesse = generate_random_value();
        sensor_data.bpm = generate_random_value();
        sensor_data.altitude = generate_random_value();

        ESP_LOGI(TAG, "Nouvelles données : vitesse=%d, bpm=%d, altitude=%d",
             sensor_data.vitesse, (int)result.heart_bpm, sensor_data.altitude);

    if (notify_handle != 0) { // Vérifie si le handle est valide
            char json_data[128];
            snprintf(json_data, sizeof(json_data),
                     "{\"vitesse\": %d, \"bpm\": %d, \"altitude\": %d}",
                     sensor_data.vitesse, (int)result.heart_bpm, sensor_data.altitude);

            struct os_mbuf *om = ble_hs_mbuf_from_flat(json_data, strlen(json_data));
            if (om) {
                int rc = ble_gattc_notify_custom(0, notify_handle, om);
                if (rc != 0) {
                    ESP_LOGE(TAG, "Erreur d'envoi de notification : %d", rc);
                }
            }
        } //de if à ici pour envoie automatique*/
        vTaskDelay(pdMS_TO_TICKS(1000)); // Mise à jour toutes les secondes
    }
}

// GATT : Lecture des données (format JSON)
static int device_read(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    char json_data[128];
    max30100_data_t result = {};
    ESP_ERROR_CHECK(max30100_update(&max30100, &result));

    // Formater les données en JSON
snprintf(json_data, sizeof(json_data),
         "{\n"
         "  \"vitesse\": %d,\n"
         "  \"bpm\": %d,\n"
         "  \"altitude\": %d\n"
         "}",
             sensor_data.vitesse, (int)result.heart_bpm, sensor_data.altitude);

    ESP_LOGI(TAG, "Données lues par le client : %s", json_data);

    os_mbuf_append(ctxt->om, json_data, strlen(json_data)); // Envoyer les données JSON au client
    return 0;
}

// Définition des services et caractéristiques BLE
static const struct ble_gatt_svc_def gatt_svcs[] = {
    {.type = BLE_GATT_SVC_TYPE_PRIMARY,
     .uuid = BLE_UUID16_DECLARE(0x1111), // UUID du service (service générique)
     .characteristics = (struct ble_gatt_chr_def[]){
         {.uuid = BLE_UUID16_DECLARE(0x2222), // UUID de la caractéristique lecture
          .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY, // Activer les notifications BLE_GATT_CHR_F_NOTIFY .val_handle = &notify_handle,
            .val_handle = &notify_handle,
          .access_cb = device_read}, // Lecture des données JSON
         {0}}},                      // Fin des caractéristiques
    {0}};                           // Fin des services

// Gestion des événements BLE GAP
static int ble_gap_event(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        ESP_LOGI(TAG, "Connexion établie, status: %s", event->connect.status == 0 ? "OK" : "FAILED");
        if (event->connect.status != 0) {
            ble_app_advertise(); // Appel à `ble_app_advertise`
        }
        break;
    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "Déconnexion");
        ble_app_advertise(); // Appel à `ble_app_advertise`
        break;
    default:
        break;
    }
    return 0;
}

// Publicité BLE
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

// Synchronisation BLE
void ble_app_on_sync(void) {
    ble_hs_id_infer_auto(0, &ble_addr_type);
    ble_app_advertise();
}

// Tâche principale BLE
void host_task(void *param) {
    nimble_port_run();
}

void app_main() {
    nvs_flash_init();
    nimble_port_init();
    ble_svc_gap_device_name_set("BLE-Random"); // Nom du périphérique BLE
    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_gatts_count_cfg(gatt_svcs);
    ble_gatts_add_svcs(gatt_svcs);
    ble_hs_cfg.sync_cb = ble_app_on_sync;
    nimble_port_freertos_init(host_task);


    //Init I2C_NUM_0
    ESP_ERROR_CHECK(i2c_master_init(I2C_PORT));
    //Init sensor at I2C_NUM_0
    ESP_ERROR_CHECK(max30100_init( &max30100, I2C_PORT,
                   MAX30100_DEFAULT_OPERATING_MODE,
                   MAX30100_DEFAULT_SAMPLING_RATE,
                   MAX30100_DEFAULT_LED_PULSE_WIDTH,
                   MAX30100_DEFAULT_IR_LED_CURRENT,
                   MAX30100_DEFAULT_START_RED_LED_CURRENT,
                   MAX30100_DEFAULT_MEAN_FILTER_SIZE,
                   MAX30100_DEFAULT_PULSE_BPM_SAMPLE_SIZE,
                   true, false ));

    //Start test task
    xTaskCreate(get_bpm, "Get BPM", 8192, NULL, 1, NULL);
    // Démarrer la tâche pour mettre à jour les données JSON
    xTaskCreate(update_sensor_data_task, "update_sensor_data_task", 4096, NULL, 5, NULL);


}
