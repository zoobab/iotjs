/* Copyright 2015 Samsung Electronics Co., Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#if defined(__NUTTX__)


#include "iotjs_module_gpio.h"

#include <unistd.h>
#include <fcntl.h>
#include <nuttx/gpio.h>
#include <sys/ioctl.h>

#define GPIO_MAX_PINNO 10


//////// Except from nuttx/arch/arm/src/stm32/stm32_gpio.h /////
/* Each port bit of the general-purpose I/O (GPIO) ports can be individually
 * configured
 * by software in several modes:
 *
 *  - Input floating
 *  - Input pull-up
 *  - Input-pull-down
 *  - Output open-drain with pull-up or pull-down capability
 *  - Output push-pull with pull-up or pull-down capability
 *  - Alternate function push-pull with pull-up or pull-down capability
 *  - Alternate function open-drain with pull-up or pull-down capability
 *  - Analog
 *
 * 20-bit Encoding:       1111 1111 1100 0000 0000
 *                        9876 5432 1098 7654 3210
 *                        ---- ---- ---- ---- ----
 * Inputs:                MMUU .... ...X PPPP BBBB
 * Outputs:               MMUU .... FFOV PPPP BBBB
 * Alternate Functions:   MMUU AAAA FFO. PPPP BBBB
 * Analog:                MM.. .... .... PPPP BBBB
 */

/* Mode:
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * MM.. .... .... .... ....
 */

#define GPIO_MODE_SHIFT               (18)
/* Bits 18-19: GPIO port mode */
#define GPIO_MODE_MASK                (3 << GPIO_MODE_SHIFT)
#  define GPIO_INPUT                  (0 << GPIO_MODE_SHIFT)
/* Input mode */
#  define GPIO_OUTPUT                 (1 << GPIO_MODE_SHIFT)
/* General purpose output mode */
#  define GPIO_ALT                    (2 << GPIO_MODE_SHIFT)
/* Alternate function mode */
#  define GPIO_ANALOG                 (3 << GPIO_MODE_SHIFT)
/* Analog mode */

/* Input/output pull-ups/downs (not used with analog):
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * ..UU .... .... .... ....
 */

#define GPIO_PUPD_SHIFT               (16)
/* Bits 16-17: Pull-up/pull down */
#define GPIO_PUPD_MASK                (3 << GPIO_PUPD_SHIFT)
#  define GPIO_FLOAT                  (0 << GPIO_PUPD_SHIFT)
/* No pull-up, pull-down */
#  define GPIO_PULLUP                 (1 << GPIO_PUPD_SHIFT)
/* Pull-up */
#  define GPIO_PULLDOWN               (2 << GPIO_PUPD_SHIFT)
/* Pull-down */

/* Alternate Functions:
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * .... AAAA .... .... ....
 */

#define GPIO_AF_SHIFT                 (12)
/* Bits 12-15: Alternate function */
#define GPIO_AF_MASK                  (15 << GPIO_AF_SHIFT)
#  define GPIO_AF(n)                  ((n) << GPIO_AF_SHIFT)
#  define GPIO_AF0                    (0 << GPIO_AF_SHIFT)
#  define GPIO_AF1                    (1 << GPIO_AF_SHIFT)
#  define GPIO_AF2                    (2 << GPIO_AF_SHIFT)
#  define GPIO_AF3                    (3 << GPIO_AF_SHIFT)
#  define GPIO_AF4                    (4 << GPIO_AF_SHIFT)
#  define GPIO_AF5                    (5 << GPIO_AF_SHIFT)
#  define GPIO_AF6                    (6 << GPIO_AF_SHIFT)
#  define GPIO_AF7                    (7 << GPIO_AF_SHIFT)
#  define GPIO_AF8                    (8 << GPIO_AF_SHIFT)
#  define GPIO_AF9                    (9 << GPIO_AF_SHIFT)
#  define GPIO_AF10                   (10 << GPIO_AF_SHIFT)
#  define GPIO_AF11                   (11 << GPIO_AF_SHIFT)
#  define GPIO_AF12                   (12 << GPIO_AF_SHIFT)
#  define GPIO_AF13                   (13 << GPIO_AF_SHIFT)
#  define GPIO_AF14                   (14 << GPIO_AF_SHIFT)
#  define GPIO_AF15                   (15 << GPIO_AF_SHIFT)

/* Output/Alt function frequency selection:
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * .... .... FF.. .... ....
 */

#define GPIO_SPEED_SHIFT              (10)
/* Bits 10-11: GPIO frequency selection */
#define GPIO_SPEED_MASK               (3 << GPIO_SPEED_SHIFT)
#if defined(CONFIG_STM32_STM32L15XX)
#  define GPIO_SPEED_400KHz           (0 << GPIO_SPEED_SHIFT)
/* 400 kHz Very low speed output */
#  define GPIO_SPEED_2MHz             (1 << GPIO_SPEED_SHIFT)
/* 2 MHz Low speed output */
#  define GPIO_SPEED_10MHz            (2 << GPIO_SPEED_SHIFT)
/* 10 MHz Medium speed output */
#  define GPIO_SPEED_40MHz            (3 << GPIO_SPEED_SHIFT)
/* 40 MHz High speed output */
#else
#  define GPIO_SPEED_2MHz             (0 << GPIO_SPEED_SHIFT)
/* 2 MHz Low speed output */
#  define GPIO_SPEED_25MHz            (1 << GPIO_SPEED_SHIFT)
/* 25 MHz Medium speed output */
#  define GPIO_SPEED_50MHz            (2 << GPIO_SPEED_SHIFT)
/* 50 MHz Fast speed output  */
#ifndef CONFIG_STM32_STM32F30XX
#  define GPIO_SPEED_100MHz           (3 << GPIO_SPEED_SHIFT)
/* 100 MHz High speed output */
#endif
#endif

/* Output/Alt function type selection:
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * .... .... ..O. .... ....
 */

#define GPIO_OPENDRAIN                (1 << 9)
/* Bit9: 1=Open-drain output */
#define GPIO_PUSHPULL                 (0)
/* Bit9: 0=Push-pull output */

/* If the pin is a GPIO digital output, then this identifies the initial
 * output value.
 * If the pin is an input, this bit is overloaded to provide the qualifier to
 * distinquish input pull-up and -down:
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * .... .... ...V .... ....
 */

#define GPIO_OUTPUT_SET               (1 << 8)
/* Bit 8: If output, inital value of output */
#define GPIO_OUTPUT_CLEAR             (0)

/* External interrupt selection (GPIO inputs only):
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * .... .... ...X .... ....
 */

#define GPIO_EXTI                     (1 << 8)
/* Bit 8: Configure as EXTI interrupt */

/* This identifies the GPIO port:
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * .... .... .... PPPP ....
 */

#define GPIO_PORT_SHIFT               (4)
/* Bit 4-7:  Port number */
#define GPIO_PORT_MASK                (15 << GPIO_PORT_SHIFT)
#  define GPIO_PORTA                  (0 << GPIO_PORT_SHIFT)     /*   GPIOA */
#  define GPIO_PORTB                  (1 << GPIO_PORT_SHIFT)     /*   GPIOB */
#  define GPIO_PORTC                  (2 << GPIO_PORT_SHIFT)     /*   GPIOC */
#  define GPIO_PORTD                  (3 << GPIO_PORT_SHIFT)     /*   GPIOD */
#  define GPIO_PORTE                  (4 << GPIO_PORT_SHIFT)     /*   GPIOE */
#if defined (CONFIG_STM32_STM32L15XX)
#  define GPIO_PORTH                  (5 << GPIO_PORT_SHIFT)     /*   GPIOH */
#  define GPIO_PORTF                  (6 << GPIO_PORT_SHIFT)     /*   GPIOF */
#  define GPIO_PORTG                  (7 << GPIO_PORT_SHIFT)     /*   GPIOG */
#else
#  define GPIO_PORTF                  (5 << GPIO_PORT_SHIFT)     /*   GPIOF */
#  define GPIO_PORTG                  (6 << GPIO_PORT_SHIFT)     /*   GPIOG */
#  define GPIO_PORTH                  (7 << GPIO_PORT_SHIFT)     /*   GPIOH */
#  define GPIO_PORTI                  (8 << GPIO_PORT_SHIFT)     /*   GPIOI */
#  define GPIO_PORTJ                  (9 << GPIO_PORT_SHIFT)     /*   GPIOJ */
#  define GPIO_PORTK                  (10 << GPIO_PORT_SHIFT)    /*   GPIOK */
#endif

/* This identifies the bit in the port:
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * .... .... .... .... BBBB
 */

#define GPIO_PIN_SHIFT                (0)
/* Bits 0-3: GPIO number: 0-15 */
#define GPIO_PIN_MASK                 (15 << GPIO_PIN_SHIFT)
#  define GPIO_PIN0                   (0 << GPIO_PIN_SHIFT)
#  define GPIO_PIN1                   (1 << GPIO_PIN_SHIFT)
#  define GPIO_PIN2                   (2 << GPIO_PIN_SHIFT)
#  define GPIO_PIN3                   (3 << GPIO_PIN_SHIFT)
#  define GPIO_PIN4                   (4 << GPIO_PIN_SHIFT)
#  define GPIO_PIN5                   (5 << GPIO_PIN_SHIFT)
#  define GPIO_PIN6                   (6 << GPIO_PIN_SHIFT)
#  define GPIO_PIN7                   (7 << GPIO_PIN_SHIFT)
#  define GPIO_PIN8                   (8 << GPIO_PIN_SHIFT)
#  define GPIO_PIN9                   (9 << GPIO_PIN_SHIFT)
#  define GPIO_PIN10                  (10 << GPIO_PIN_SHIFT)
#  define GPIO_PIN11                  (11 << GPIO_PIN_SHIFT)
#  define GPIO_PIN12                  (12 << GPIO_PIN_SHIFT)
#  define GPIO_PIN13                  (13 << GPIO_PIN_SHIFT)
#  define GPIO_PIN14                  (14 << GPIO_PIN_SHIFT)
#  define GPIO_PIN15                  (15 << GPIO_PIN_SHIFT)

// NUTTX on STM32F4 Gpio map in IoT.js //

uint32_t gpioPin[5] = {
  0,
  (GPIO_PIN8   | GPIO_PORTA),
  (GPIO_PIN10  | GPIO_PORTA ),
  (GPIO_PIN15  | GPIO_PORTA ),
  (GPIO_PIN11  | GPIO_PORTD ) };
uint32_t gpioDirection[3] = {
  (GPIO_OUTPUT | GPIO_FLOAT),
  (GPIO_INPUT  | GPIO_PULLDOWN),
  (GPIO_OUTPUT | GPIO_PULLUP) };



namespace iotjs {


// GPIO implementeation for arm-nuttx target.
class GpioArmNuttx : public Gpio {
 public:
  explicit GpioArmNuttx(JObject& jgpio);

  static GpioArmNuttx* GetInstance();

  virtual int Initialize(GpioReqWrap* gpio_req);
  virtual int Release(GpioReqWrap* gpio_req);
  virtual int SetPin(GpioReqWrap* gpio_req);
  virtual int WritePin(GpioReqWrap* gpio_req);
  virtual int ReadPin(GpioReqWrap* gpio_req);

  bool _initialized;
  int _fd; // ioctl handle for gpio on nuttx
};



Gpio* Gpio::Create(JObject& jgpio) {
  return new GpioArmNuttx(jgpio);
}


GpioArmNuttx::GpioArmNuttx(JObject& jgpio)
    : Gpio(jgpio) {
  _fd = NULL;
  _initialized = false;
}


GpioArmNuttx* GpioArmNuttx::GetInstance()
{
  return static_cast<GpioArmNuttx*>(Gpio::GetInstance());
}


void AfterWork(uv_work_t* work_req, int status) {
  GpioArmNuttx* gpio = GpioArmNuttx::GetInstance();

  GpioReqWrap* gpio_req = reinterpret_cast<GpioReqWrap*>(work_req->data);
  GpioReqData* req_data = gpio_req->req();

  if (status) {
    req_data->result = kGpioErrSys;
  }

  JArgList jargs(2);
  jargs.Add(JVal::Number(req_data->result));

  switch (req_data->op) {
    case kGpioOpInitize:
    {
      if (req_data->result == kGpioErrOk) {
        gpio->_initialized = true;
      }
      break;
    }
    case kGpioOpRelease:
    {
      if (req_data->result == kGpioErrOk) {
        gpio->_initialized = false;
      }
      break;
    }
    case kGpioOpSetPin:
    case kGpioOpWritePin:
    {
      break;
    }
    case kGpioOpReadPin:
    {
      if (req_data->result == kGpioErrOk) {
        jargs.Add(JVal::Bool(req_data->value));
      }
      break;
    }

    case kGpioOpSetPort:
    case kGpioOpWritePort:
    case kGpioOpReadPort:
    {
      IOTJS_ASSERT(!"Not implemented");
      break;
    }
    default:
    {
      IOTJS_ASSERT(!"Unreachable");
      break;
    }
  }

  MakeCallback(gpio_req->jcallback(), *Gpio::GetJGpio(), jargs);

  delete work_req;
  delete gpio_req;
}


void InitializeWorker(uv_work_t* work_req) {
  GpioArmNuttx* gpio = GpioArmNuttx::GetInstance();
  IOTJS_ASSERT(gpio->_initialized == false);

  GpioReqWrap* gpio_req = reinterpret_cast<GpioReqWrap*>(work_req->data);
  GpioReqData* req_data = gpio_req->req();

  DDDLOG("GPIO InitializeWorker()");

  const char* devfilepath = "/dev/gpio";
  gpio->_fd = open(devfilepath, O_RDWR);
  DDDLOG("gpio> %s : fd(%d)", devfilepath, gpio->_fd);

  // Check if GPIO handle is OK.
  if (gpio->_fd) {
    req_data->result = kGpioErrOk;
  } else {
    req_data->result = kGpioErrInitialize;
  }
}


void ReleaseWorker(uv_work_t* work_req) {
  GpioArmNuttx* gpio = GpioArmNuttx::GetInstance();
  IOTJS_ASSERT(gpio->_initialized == true);

  GpioReqWrap* gpio_req = reinterpret_cast<GpioReqWrap*>(work_req->data);
  GpioReqData* req_data = gpio_req->req();

  DDDLOG("GPIO ReleaseWorker()");

  if (gpio->_fd) {
    close(gpio->_fd);
  }
  req_data->result = kGpioErrOk;
}


void SetPinWorker(uv_work_t* work_req) {
  GpioArmNuttx* gpio = GpioArmNuttx::GetInstance();
  IOTJS_ASSERT(gpio->_initialized == true);

  GpioReqWrap* gpio_req = reinterpret_cast<GpioReqWrap*>(work_req->data);
  GpioReqData* req_data = gpio_req->req();

  DDDLOG("GPIO SetPinWorker() - pin: %d, dir: %d, mode: %d",
         req_data->pin, req_data->dir, req_data->mode);
  struct gpioioctl_config_s cdata;
  cdata.port = (gpioPin[req_data->pin] | gpioDirection[req_data->dir]);

  if (!ioctl(gpio->_fd, GPIOIOCTL_CONFIG, (long unsigned int)&cdata)) {
    req_data->result = kGpioErrOk;
  } else {
    req_data->result = kGpioErrSys;
  }
}



void WritePinWorker(uv_work_t* work_req) {
  GpioArmNuttx* gpio = GpioArmNuttx::GetInstance();
  IOTJS_ASSERT(gpio->_initialized == true);

  GpioReqWrap* gpio_req = reinterpret_cast<GpioReqWrap*>(work_req->data);
  GpioReqData* req_data = gpio_req->req();

  DDDLOG("GPIO WritePinWorker() - pin: %d, value: %d",
         req_data->pin, req_data->value);

  struct gpioioctl_write_s wdata;
  wdata.port = gpioPin[req_data->pin];
  wdata.data = req_data->value;

  if (!ioctl(gpio->_fd, GPIOIOCTL_WRITE, (long unsigned int)&wdata)) {
    req_data->result = kGpioErrOk;
  } else {
    req_data->result = kGpioErrSys;
  }
}



void ReadPinWorker(uv_work_t* work_req) {
  GpioArmNuttx* gpio = GpioArmNuttx::GetInstance();
  IOTJS_ASSERT(gpio->_initialized == true);

  GpioReqWrap* gpio_req = reinterpret_cast<GpioReqWrap*>(work_req->data);
  GpioReqData* req_data = gpio_req->req();

  DDDLOG("GPIO ReadPinWorker() - pin: %d" ,req_data->pin);

  struct gpioioctl_read_s rdata;
  int ret;
  rdata.port = gpioPin[req_data->pin];

  if (!ioctl(gpio->_fd, GPIOIOCTL_READ, (long unsigned int)&rdata)) {
    req_data->value = rdata.data;
    req_data->result = kGpioErrOk;
  } else {
    req_data->result = kGpioErrSys;
  }
}


#define GPIO_NUTTX_IMPL_TEMPLATE(op, initialized) \
  do { \
    GpioArmNuttx* gpio = GpioArmNuttx::GetInstance(); \
    IOTJS_ASSERT(gpio->_initialized == initialized); \
    Environment* env = Environment::GetEnv(); \
    uv_work_t* req = new uv_work_t; \
    req->data = reinterpret_cast<void*>(gpio_req); \
    uv_queue_work(env->loop(), req, op ## Worker, AfterWork); \
  } while (0)


int GpioArmNuttx::Initialize(GpioReqWrap* gpio_req) {
  GPIO_NUTTX_IMPL_TEMPLATE(Initialize, false);
  return 0;
}


int GpioArmNuttx::Release(GpioReqWrap* gpio_req) {
  GPIO_NUTTX_IMPL_TEMPLATE(Release, true);
  return 0;
}


int GpioArmNuttx::SetPin(GpioReqWrap* gpio_req) {
  GPIO_NUTTX_IMPL_TEMPLATE(SetPin, true);
  return 0;
}


int GpioArmNuttx::WritePin(GpioReqWrap* gpio_req) {
  GPIO_NUTTX_IMPL_TEMPLATE(WritePin, true);
  return 0;
}


int GpioArmNuttx::ReadPin(GpioReqWrap* gpio_req) {
  GPIO_NUTTX_IMPL_TEMPLATE(ReadPin, true);
  return 0;
}


} // namespace iotjs

#endif // __NUTTX__
