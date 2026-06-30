#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/printk.h>

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#define TAM_PILHA_ADC          768
#define TAM_PILHA_ACEL         1024

#define PRIORIDADE_ADC         5
#define PRIORIDADE_ACEL        5

#define PERIODO_ADC_MS         500
#define PERIODO_ACEL_MS        1000

#define BOTAO_NODE             DT_NODELABEL(user_button_0)

#define RESOLUCAO_ADC          8
#define GANHO_ADC              ADC_GAIN_1
#define REFERENCIA_ADC         ADC_REF_INTERNAL
#define TEMPO_AQUISICAO_ADC    ADC_ACQ_TIME_DEFAULT
#define CANAL_ADC              9
#define TENSAO_REF_MV          3300

static const struct gpio_dt_spec botao = GPIO_DT_SPEC_GET(BOTAO_NODE, gpios);
static struct gpio_callback callback_botao;

static int16_t amostra_adc;

static const struct device *const dispositivo_acel =
    DEVICE_DT_GET(DT_NODELABEL(mma8451q));

typedef enum {
    MODO_TUDO = 0,
    MODO_SOMENTE_ADC = 1
} modo_sistema_t;

static volatile modo_sistema_t modo_atual = MODO_TUDO;

void rotina_adc(void *a, void *b, void *c);
void rotina_acel(void *a, void *b, void *c);

K_THREAD_DEFINE(thread_adc_id,
                TAM_PILHA_ADC,
                rotina_adc,
                NULL, NULL, NULL,
                PRIORIDADE_ADC,
                0,
                0);

K_THREAD_DEFINE(thread_acel_id,
                TAM_PILHA_ACEL,
                rotina_acel,
                NULL, NULL, NULL,
                PRIORIDADE_ACEL,
                0,
                0);

static void interrupcao_botao(const struct device *dev,
                              struct gpio_callback *cb,
                              uint32_t pins)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);

    if (modo_atual == MODO_TUDO) {
        modo_atual = MODO_SOMENTE_ADC;
        k_thread_suspend(thread_acel_id);
        printk("Modo alterado: somente ADC\n");
    } else {
        modo_atual = MODO_TUDO;
        k_thread_resume(thread_acel_id);
        printk("Modo alterado: ADC + acelerometro\n");
    }
}

static void configurar_botao(void)
{
    int erro;

    if (!gpio_is_ready_dt(&botao)) {
        printk("Falha: botao indisponivel\n");
        return;
    }

    erro = gpio_pin_configure_dt(&botao, GPIO_INPUT | GPIO_PULL_UP);
    if (erro < 0) {
        printk("Falha ao configurar botao: %d\n", erro);
        return;
    }

    erro = gpio_pin_interrupt_configure_dt(&botao, GPIO_INT_EDGE_FALLING);
    if (erro < 0) {
        printk("Falha ao ativar interrupcao do botao: %d\n", erro);
        return;
    }

    gpio_init_callback(&callback_botao, interrupcao_botao, BIT(botao.pin));
    gpio_add_callback(botao.port, &callback_botao);

    printk("Botao pronto\n");
}

void rotina_adc(void *a, void *b, void *c)
{
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);

    const struct device *adc0 = DEVICE_DT_GET(DT_NODELABEL(adc0));

    if (!device_is_ready(adc0)) {
        printk("Falha: ADC0 indisponivel\n");
        return;
    }

    struct adc_channel_cfg canal_config = {
        .gain = GANHO_ADC,
        .reference = REFERENCIA_ADC,
        .acquisition_time = TEMPO_AQUISICAO_ADC,
        .channel_id = CANAL_ADC,
        .differential = 0,
    };

    ARG_UNUSED(canal_config);

    struct adc_sequence sequencia = {
        .channels = BIT(CANAL_ADC),
        .buffer = &amostra_adc,
        .buffer_size = sizeof(amostra_adc),
        .resolution = RESOLUCAO_ADC,
    };

    while (1) {
        int erro = adc_read(adc0, &sequencia);

        if (erro < 0) {
            printk("Erro ADC: %d\n", erro);
        } else {
            int32_t tensao_mv = amostra_adc;

            adc_raw_to_millivolts(TENSAO_REF_MV,
                                  GANHO_ADC,
                                  RESOLUCAO_ADC,
                                  &tensao_mv);

            printk("ADC: bruto=%d, tensao=%d mV\n",
                   amostra_adc,
                   tensao_mv);
        }

        k_msleep(PERIODO_ADC_MS);
    }
}

void rotina_acel(void *a, void *b, void *c)
{
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);

    struct sensor_value eixo_x;
    struct sensor_value eixo_y;
    struct sensor_value eixo_z;

    uint32_t instante_ms = 0;

    if (!device_is_ready(dispositivo_acel)) {
        printk("Falha: acelerometro indisponivel\n");
        return;
    }

    printk("Acelerometro MMA8451Q pronto\n");

    k_msleep(1000);

    while (1) {
        int erro = sensor_sample_fetch(dispositivo_acel);

        if (erro < 0) {
            printk("Erro acelerometro: %d\n", erro);
            k_msleep(PERIODO_ACEL_MS);
            instante_ms += PERIODO_ACEL_MS;
            continue;
        }

        sensor_channel_get(dispositivo_acel, SENSOR_CHAN_ACCEL_X, &eixo_x);
        sensor_channel_get(dispositivo_acel, SENSOR_CHAN_ACCEL_Y, &eixo_y);
        sensor_channel_get(dispositivo_acel, SENSOR_CHAN_ACCEL_Z, &eixo_z);

        printk("t=%u ms | ax=%d.%06d | ay=%d.%06d | az=%d.%06d\n",
               instante_ms,
               eixo_x.val1, abs(eixo_x.val2),
               eixo_y.val1, abs(eixo_y.val2),
               eixo_z.val1, abs(eixo_z.val2));

        k_msleep(PERIODO_ACEL_MS);
        instante_ms += PERIODO_ACEL_MS;
    }
}

int main(void)
{
    printk("\n");
    printk("PSI3441 - Atividade 6\n");
    printk("Modo inicial: ADC + acelerometro\n");
    printk("Pressione o botao para alternar o modo\n");
    printk("\n");

    configurar_botao();

    return 0;
}