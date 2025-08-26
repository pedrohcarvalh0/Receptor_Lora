#include <stdio.h>
#include "pico/stdlib.h"
#include <stdlib.h>
#include <string.h>
#include "lib/lora_sx1276.h"
#include "pico/binary_info.h"

// Estrutura para armazenar dados dos sensores AHT20 e BH1750
typedef struct {
    uint16_t lux;           // Luminosidade do BH1750
    float temperature;      // Temperatura do AHT20
    float humidity;         // Umidade do AHT20
    int16_t rssi;
    int8_t snr;
    uint32_t timestamp;
} sensor_data_t;

int parse_sensor_data(const char* payload, sensor_data_t* data) {
    // Formato esperado: "Lux: %d, Temperatura: %s, Umidade: %s\n"
    char temp_str[10], humidity_str[10];
    int parsed = sscanf(payload, "Lux: %hu, Temperatura: %9s Umidade: %9s",
                       &data->lux, temp_str, humidity_str);
    
    if (parsed == 3) {
        // Converter strings de temperatura e umidade para float
        // Remove 'C' da temperatura e '%' da umidade
        data->temperature = atof(temp_str); // atof ignora caracteres não numéricos no final
        data->humidity = atof(humidity_str);
        
        data->timestamp = to_ms_since_boot(get_absolute_time());
        return 1; // Sucesso
    }
    return 0; // Erro no parsing
}

void display_sensor_data(const sensor_data_t* data) {
    printf("\n=== DADOS SENSORES RECEBIDOS ===\n");
    printf("Luminosidade: %u lux\n", data->lux);
    printf("Temperatura: %.1f°C\n", data->temperature);
    printf("Umidade: %.1f%%\n", data->humidity);
    printf("Qualidade do sinal:\n");
    printf("  RSSI: %d dBm | SNR: %d dB\n", data->rssi, data->snr);
    
    // Interpretação dos valores

    // OBS: A interpretação dos valores é feita com base de prâmetros ajustados diretamente no código,
    // logo devem ser adaptatos para difrentes situações e ambientes
    printf("Interpretação:\n");
    if (data->lux < 10) {
        printf("  Luminosidade: Muito escuro\n");
    } else if (data->lux < 100) {
        printf("  Luminosidade: Escuro\n");
    } else if (data->lux < 1000) {
        printf("  Luminosidade: Ambiente interno\n");
    } else if (data->lux < 10000) {
        printf("  Luminosidade: Ambiente claro\n");
    } else {
        printf("  Luminosidade: Muito claro/Sol direto\n");
    }
    
    if (data->temperature < 15) {
        printf("  Temperatura: Frio\n");
    } else if (data->temperature < 25) {
        printf("  Temperatura: Agradável\n");
    } else if (data->temperature < 35) {
        printf("  Temperatura: Quente\n");
    } else {
        printf("  Temperatura: Muito quente\n");
    }
    
    if (data->humidity < 30) {
        printf("  Umidade: Seco\n");
    } else if (data->humidity < 60) {
        printf("  Umidade: Confortável\n");
    } else if (data->humidity < 80) {
        printf("  Umidade: Úmido\n");
    } else {
        printf("  Umidade: Muito úmido\n");
    }
    printf("===============================\n\n");
}

int main() {
    stdio_init_all();
    sleep_ms(2000); // Aguarda inicialização do USB serial
    
    printf("\n=== RECEPTOR LORA SENSORES AMBIENTAIS ===\n");
    printf("Sensores: AHT20 (Temp/Umidade) + BH1750 (Luminosidade)\n");
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
    printf("Frequência: 915 MHz\n");
    printf("Pressione Ctrl+C para sair.\n\n");
    
    // Buffer para receber dados
    uint8_t rx_buffer[256];
    sensor_data_t sensor_data;
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
                
                if (parse_sensor_data((char*)rx_buffer, &sensor_data)) {
                    sensor_data.rssi = rssi;
                    sensor_data.snr = snr;
                    
                    display_sensor_data(&sensor_data);
                    
                    packet_count++;
                } else {
                    printf("ERRO: Falha ao parsear dados dos sensores\n");
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
