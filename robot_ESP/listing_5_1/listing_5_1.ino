//Подключаем библиотеку.
#include <Preferences.h>
//Создаем объект хранилище.
Preferences preferences;
void setup() {
  //Открытие serial порта
  // на скорости 115200 Бит/сек (Бод).
  Serial.begin(115200);
  Serial.println("Open Storage");
  // Открываем хранилище с именем storage.
  // второй параметр false - режим записи/чтения.
  // Длина имене не более 15 симв.
  Serial.println(preferences.begin("storage", false));
  // Удаление всех хранилищ
  //preferences.clear();
  // Удаление отдельного ключа
  //preferences.remove("counter");
}
//Бесконечный цикл loop.
void loop() {
  //Если в порту есть данные
  if (Serial.available())
  {
    //Читаем один байт из порта.
    byte Sdata = Serial.read();
    //  Если код введенной буквы сооьветствует 
    // значению кода цифр от 0 до 9 (код 48-57)
    if ((Sdata > 47) && (Sdata < 58))
    {
      // Сохранить значение времени от начала работы программы
      //под именем "serialDataStorage"
      Serial.print("Writing...");
      unsigned long serialDataStorage1 = millis();
      Serial.println(serialDataStorage1);
     preferences.putULong("sDS", serialDataStorage1);
    }
    else
    {
      if (Sdata == 'a')
      {
        //Если введена буква 'a'
        // Прочитать значение ключа с указанным именем,
        //если ключа нет, вернуть 0
        unsigned long serialDataStorage2 =
        preferences.getULong("sDS");
        Serial.print("Read...");Serial.println(serialDataStorage2);
       }
    }
  }
}
