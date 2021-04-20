#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <avr/io.h>
#include <avr/interrupt.h>


#define VENTSTARTVALUE 3400
#define RESCORRECTION 200
#define THERMISTOR_PIN PB3 //PIN термистора
#define OPTRON_PIN PB1 //PIN вентилятора
#define ENGINE_PIN PB4 //PIN статуса двигателя
#define COUNTSAMPLES 10

float BALANCE_RESISTOR = 10000.0; //сопротивление балансного резистора
int samples[COUNTSAMPLES];
bool globalPower;


ISR(PCINT0_vect)
{
  cli(); //отключаем прерывания
  PCMSK &= ~_BV(ENGINE_PIN); //Больше не ловим прерывания
}

ISR(WDT_vect) {}

//Двигатель не работает, спим
void activatesleep ()
{
  GIMSK = _BV(PCIE); // Включить Pin Change прерывания
  PCMSK |= _BV(ENGINE_PIN); //Прерывания ловим на порту ENGINE
  ADCSRA &= ~_BV(ADEN); //выключение ADC
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sei(); //включение прерывания
  sleep_cpu();  // начали спать
  //просыпаемся тут
  sleep_disable();
  ADCSRA |= _BV(ADEN); //Включение ADC
}


void setup(void) {
  pinMode(OPTRON_PIN, OUTPUT);
  pinMode(ENGINE_PIN, INPUT_PULLUP);
  digitalWrite(OPTRON_PIN, LOW);
}

float getAvgTermo()
{
  int i;
  float average;
  for (i = 0; i < COUNTSAMPLES; i++) {
    samples[i] = analogRead(THERMISTOR_PIN);
    delay(75);
  }
  //среднее значение
  average = 0;
  for (i = 0; i < COUNTSAMPLES; i++) {
    average += samples[i];
  }
  average /= COUNTSAMPLES;
  average = 1023 / average - 1;
  average = BALANCE_RESISTOR / average;
  return average;
}

uint8_t enginestatus()
{
  long i;
  int enginestatus = 0;
  for (i = 0; i < 100000; i++) {
    if (digitalRead(ENGINE_PIN) == 0)
    {
      enginestatus = 1;
      break;
    }
  }
  return enginestatus;
}

void loop(void) {
  int isengine;
  float avgTermo;

  isengine = enginestatus();
  if ( isengine == 0)
  {
    digitalWrite(OPTRON_PIN, LOW);
    activatesleep();
  }
  avgTermo = getAvgTermo();

  //Обрыв термистора
  if (avgTermo == INFINITY)
  {
    avgTermo = 30000;
  }

  if (globalPower == 1)
  {
    avgTermo = avgTermo - RESCORRECTION;
  }
  if (avgTermo < VENTSTARTVALUE)
  {
    digitalWrite(OPTRON_PIN, HIGH);
    globalPower = 1;
  }
  else
  {
    digitalWrite(OPTRON_PIN, LOW);
    globalPower = 0;
  }
}
