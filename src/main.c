#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/printk.h>

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

/*
 * Atividade 6 - Threads
 *
 * Thread ADC:
 * - leitura a cada 500 ms
 *
 * Thread Acelerômetro:
 * - leitura a cada 1000 ms
 *
 * Botão:
 * - alterna entre modo ADC e modo completo
 */

// -----------------------------------------------------------------------------
// Configurações gerais das threads
// -----------------------------------------------------------------------------

#define ADC_THREAD_STACK_SIZE       768
#define ACCEL_THREAD_STACK_SIZE     1024

#define ADC_THREAD_PRIORITY         5
#define ACCEL_THREAD_PRIORITY       5

#define ADC_PERIOD_MS               500
#define ACCEL_PERIOD_MS             1000

// -----------------------------------------------------------------------------
// Botão
// -----------------------------------------------------------------------------

#define BUTTON_NODE DT_NODELABEL(user_button_0)

static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(BUTTON_NODE, gpios);
static struct gpio_callback button_callback;

// -----------------------------------------------------------------------------
// ADC
// -----------------------------------------------------------------------------

#define ADC_RESOLUTION              8
#define ADC_GAIN                    ADC_GAIN_1
#define ADC_REFERENCE               ADC_REF_INTERNAL
#define ADC_ACQUISITION_TIME        ADC_ACQ_TIME_DEFAULT
#define ADC_CHANNEL_ID              9
#define ADC_VREF_MV                 3300

static int16_t adc_sample_buffer;

// -----------------------------------------------------------------------------
// Acelerômetro
// -----------------------------------------------------------------------------

static const struct device *const accel_dev =
    DEVICE_DT_GET(DT_NODELABEL(mma8451q));

// -----------------------------------------------------------------------------
// Controle de modo
// -----------------------------------------------------------------------------

typedef enum {
    MODE_COMPLETE = 0,
    MODE_ADC_ONLY = 1
} system_mode_t;

static volatile system_mode_t current_mode = MODE_COMPLETE;

// -----------------------------------------------------------------------------
// Protótipos das threads
// -----------------------------------------------------------------------------

void adc_thread(void *p1, void *p2, void *p3);
void accel_thread(void *p1, void *p2, void *p3);

// -----------------------------------------------------------------------------
// Definição das threads
// -----------------------------------------------------------------------------

K_THREAD_DEFINE(adc_thread_id,
                ADC_THREAD_STACK_SIZE,
                adc_thread,
                NULL, NULL, NULL,
                ADC_THREAD_PRIORITY,
                0,
                0);

K_THREAD_DEFINE(accel_thread_id,
                ACCEL_THREAD_STACK_SIZE,
                accel_thread,
                NULL, NULL, NULL,
                ACCEL_THREAD_PRIORITY,
                0,
                0);

// -----------------------------------------------------------------------------
// ISR do botão
// -----------------------------------------------------------------------------

static void button_isr(const struct device *dev,
                       struct gpio_callback *cb,
                       uint32_t pins)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);

    if (current_mode == MODE_COMPLETE) {
        current_mode = MODE_ADC_ONLY;
        k_thread_suspend(accel_thread_id);
    } else {
        current_mode = MODE_COMPLETE;
        k_thread_resume(accel_thread_id);
    }
}

// -----------------------------------------------------------------------------
// Inicialização do botão
// -----------------------------------------------------------------------------

static void init_button(void)
{
    int ret;

    if (!gpio_is_ready_dt(&button)) {
        printk("Erro: botao nao esta pronto\n");
        return;
    }

    ret = gpio_pin_configure_dt(&button, GPIO_INPUT | GPIO_PULL_UP);
    if (ret < 0) {
        printk("Erro ao configurar botao: %d\n", ret);
        return;
    }

    ret = gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_FALLING);
    if (ret < 0) {
        printk("Erro ao configurar interrupcao do botao: %d\n", ret);
        return;
    }

    gpio_init_callback(&button_callback, button_isr, BIT(button.pin));
    gpio_add_callback(button.port, &button_callback);

    printk("Botao configurado com interrupcao\n");
}

// -----------------------------------------------------------------------------
// Thread do ADC
// -----------------------------------------------------------------------------

void adc_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    const struct device *adc_dev = DEVICE_DT_GET(DT_NODELABEL(adc0));

    if (!device_is_ready(adc_dev)) {
        printk("Erro: ADC nao esta pronto\n");
        return;
    }

    struct adc_channel_cfg adc_channel_config = {
        .gain = ADC_GAIN,
        .reference = ADC_REFERENCE,
        .acquisition_time = ADC_ACQUISITION_TIME,
        .channel_id = ADC_CHANNEL_ID,
        .differential = 0,
    };

    /*
     * Dependendo da configuração do projeto/Device Tree, esta chamada pode ser
     * necessária. Se o canal já estiver configurado pelo ambiente, pode ser
     * mantida comentada.
     */
    /*
    if (adc_channel_setup(adc_dev, &adc_channel_config) != 0) {
        printk("Erro ao configurar canal ADC\n");
        return;
    }
    */

    ARG_UNUSED(adc_channel_config);

    struct adc_sequence adc_sequence_config = {
        .channels = BIT(ADC_CHANNEL_ID),
        .buffer = &adc_sample_buffer,
        .buffer_size = sizeof(adc_sample_buffer),
        .resolution = ADC_RESOLUTION,
    };

    while (1) {
        int ret = adc_read(adc_dev, &adc_sequence_config);

        if (ret != 0) {
            printk("Falha na leitura do ADC: %d\n", ret);
        } else {
            int32_t adc_mv = adc_sample_buffer;

            adc_raw_to_millivolts(ADC_VREF_MV,
                                  ADC_GAIN,
                                  ADC_RESOLUTION,
                                  &adc_mv);

            printk("ADC: %d raw, %d mV\n",
                   adc_sample_buffer,
                   adc_mv);
        }

        k_msleep(ADC_PERIOD_MS);
    }
}

// -----------------------------------------------------------------------------
// Thread do acelerômetro
// -----------------------------------------------------------------------------

void accel_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    struct sensor_value accel_x;
    struct sensor_value accel_y;
    struct sensor_value accel_z;

    uint32_t time_ms = 0;

    printk("\n");
    printk("========================================\n");
    printk("  FRDM-KL25Z - Acelerometro MMA8451Q\n");
    printk("========================================\n");
    printk("I2C0: PTE24 (SCL), PTE25 (SDA)\n");
    printk("========================================\n\n");

    if (!device_is_ready(accel_dev)) {
        printk("Erro: acelerometro nao esta pronto\n");
        return;
    }

    printk("Acelerometro inicializado com sucesso\n");
    printk("Leituras a cada 1000 ms\n\n");

    k_msleep(1000);

    while (1) {
        int ret = sensor_sample_fetch(accel_dev);

        if (ret != 0) {
            printk("Erro ao ler acelerometro: %d\n", ret);
            k_msleep(ACCEL_PERIOD_MS);
            time_ms += ACCEL_PERIOD_MS;
            continue;
        }

        sensor_channel_get(accel_dev, SENSOR_CHAN_ACCEL_X, &accel_x);
        sensor_channel_get(accel_dev, SENSOR_CHAN_ACCEL_Y, &accel_y);
        sensor_channel_get(accel_dev, SENSOR_CHAN_ACCEL_Z, &accel_z);

        printk("T: %u ms, X: %d.%06d, Y: %d.%06d, Z: %d.%06d\n",
               time_ms,
               accel_x.val1, abs(accel_x.val2),
               accel_y.val1, abs(accel_y.val2),
               accel_z.val1, abs(accel_z.val2));

        k_msleep(ACCEL_PERIOD_MS);
        time_ms += ACCEL_PERIOD_MS;
    }
}

// -----------------------------------------------------------------------------
// Main
// -----------------------------------------------------------------------------

int main(void)
{
    printk("\n");
    printk("========================================\n");
    printk("  PSI3441 - Atividade 6 - Threads\n");
    printk("========================================\n");
    printk("Modo inicial: COMPLETO\n");
    printk("Botao alterna entre:\n");
    printk("- Modo ADC: somente ADC\n");
    printk("- Modo Completo: ADC + acelerometro\n");
    printk("========================================\n\n");

    init_button();

    return 0;
}
