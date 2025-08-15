#include <stdio.h>
#include "pico/stdlib.h"
#include <string.h>
#include "lib/lora_sx1276.h"
#include "pico/binary_info.h"

// Estrutura para armazenar dados do MPU6050
typedef struct {
    int16_t accel_x, accel_y, accel_z;
    int16_t gyro_x, gyro_y, gyro_z;
    int16_t rssi;
    int8_t snr;
    uint32_t timestamp;
} mpu6050_data_t;

// Função para parsear os dados recebidos
int parse_mpu6050_data(const char* payload, mpu6050_data_t* data) {
    // Formato esperado: "Ax: %d, Ay: %d, Az: %d\nGx: %d, Gy: %d, Gz: %d\n"
    int parsed = sscanf(payload, "Ax: %hd, Ay: %hd, Az: %hd\nGx: %hd, Gy: %hd, Gz: %hd",
                       &data->accel_x, &data->accel_y, &data->accel_z,
                       &data->gyro_x, &data->gyro_y, &data->gyro_z);
    
    if (parsed == 6) {
        data->timestamp = to_ms_since_boot(get_absolute_time());
        return 1; // Sucesso
    }
    return 0; // Erro no parsing
}

// Função para exibir dados formatados
void display_mpu6050_data(const mpu6050_data_t* data) {
    printf("\n=== DADOS MPU6050 RECEBIDOS ===\n");
    printf("Timestamp: %lu ms\n", data->timestamp);
    printf("Aceleração (raw):\n");
    printf("  X: %6d | Y: %6d | Z: %6d\n", data->accel_x, data->accel_y, data->accel_z);
    printf("Giroscópio (raw):\n");
    printf("  X: %6d | Y: %6d | Z: %6d\n", data->gyro_x, data->gyro_y, data->gyro_z);
    printf("Qualidade do sinal:\n");
    printf("  RSSI: %d dBm | SNR: %d dB\n", data->rssi, data->snr);
    
    // Conversão aproximada para unidades físicas (opcional)
    printf("Aceleração (g):\n");
    printf("  X: %6.2f | Y: %6.2f | Z: %6.2f\n",
           data->accel_x / 16384.0f, data->accel_y / 16384.0f, data->accel_z / 16384.0f);
    printf("Giroscópio (°/s):\n");
    printf("  X: %6.2f | Y: %6.2f | Z: %6.2f\n",
           data->gyro_x / 131.0f, data->gyro_y / 131.0f, data->gyro_z / 131.0f);
    printf("===============================\n\n");
}

int main() {
    stdio_init_all();
    sleep_ms(2000); // Aguarda inicialização do USB serial
    
    printf("\n=== RECEPTOR LORA MPU6050 ===\n");
    printf("Iniciando receptor LoRa...\n");
    
    // Inicializar LoRa
    lora_init();
    uint8_t ver = lora_read_reg(0x42); // REG_VERSION
    printf("LoRa inicializado. Versão do chip = 0x%02X\n", ver);
    
    if (ver == 0x00 || ver == 0xFF) {
        printf("ERRO: Falha na comunicação com módulo LoRa!\n");
        printf("Verifique as conexões SPI.\n");
        while(1) {
            sleep_ms(1000);
        }
    }
    
    printf("Receptor pronto! Aguardando dados do transmissor...\n");
    printf("Frequência: 915 MHz\n\n");
    
    // Buffer para receber dados
    uint8_t rx_buffer[256];
    mpu6050_data_t sensor_data;
    uint32_t packet_count = 0;
    uint32_t error_count = 0;
    uint32_t last_status_time = 0;
    
    // Entrar em modo de recepção contínua
    lora_start_receive();
    
    while (1) {
        // Verificar se há pacote disponível
        if (lora_packet_available()) {
            int16_t rssi;
            int8_t snr;
            
            // Ler o pacote
            int bytes_received = lora_read_packet(rx_buffer, sizeof(rx_buffer) - 1, &rssi, &snr);
            
            if (bytes_received > 0) {
                // Null-terminate the string
                rx_buffer[bytes_received] = '\0';
                
                printf("Pacote recebido (%d bytes): %s", bytes_received, (char*)rx_buffer);
                
                // Parsear dados do MPU6050
                if (parse_mpu6050_data((char*)rx_buffer, &sensor_data)) {
                    sensor_data.rssi = rssi;
                    sensor_data.snr = snr;
                    
                    // Exibir dados formatados
                    display_mpu6050_data(&sensor_data);
                    
                    packet_count++;
                } else {
                    printf("ERRO: Falha ao parsear dados do MPU6050\n");
                    printf("Dados brutos: %s\n", (char*)rx_buffer);
                    error_count++;
                }
            } else if (bytes_received == -3) {
                printf("ERRO: CRC inválido no pacote recebido\n");
                error_count++;
            } else {
                printf("ERRO: Falha na recepção (código: %d)\n", bytes_received);
                error_count++;
            }
            
            // Voltar ao modo de recepção
            lora_start_receive();
        }
        
        // Exibir status periodicamente (a cada 30 segundos)
        uint32_t current_time = to_ms_since_boot(get_absolute_time());
        if (current_time - last_status_time > 30000) {
            printf("--- STATUS ---\n");
            printf("Pacotes recebidos: %lu\n", packet_count);
            printf("Erros: %lu\n", error_count);
            printf("Taxa de sucesso: %.1f%%\n",
                   packet_count > 0 ? (100.0f * packet_count) / (packet_count + error_count) : 0.0f);
            printf("Aguardando próximo pacote...\n\n");
            last_status_time = current_time;
        }
        
        sleep_ms(50);
    }
    
    return 0;
}
