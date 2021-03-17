# gkv_ros_driver
Репозиторий содержит библиотеку для приема данных в ROS, а также пример работы с ней.
Библиотека обеспечивает:
1. Прием и парсинг данных с инерциальных модулей серии ГКВ.
2. Команды:
 - проверки соединения
 - программного сброса модуля
 - установки алгоритма работы модуля
 - установки типа выдаваемого пакета
 - установки параметров наборного пакета
 - запроса ID устройства

При выборе режима выдачи данных GKV_LMP_PACKET_MODE (задан по умолчанию) устройство выдает сообщения в формате сообщений эквивалентных пакетам ГКВ.

В данном случае при приеме данных с инерциального модуля в узле, отвечающем за прием данных, формируется сообщение, соответствующее принятому пакету, и отправляется в соответствующий топик:

---------------------------------------------
Стандартный пакет данных в режиме "Коды АЦП":

Сообщение: GkvAdcData.msg

Поля сообщения:

    std_msgs/Header header
    uint16 status
    uint16 sample_counter
    int32 acceleration_adc_x
    int32 acceleration_adc_y
    int32 acceleration_adc_z
    int32 angular_rate_adc_x
    int32 angular_rate_adc_y
    int32 angular_rate_adc_z
    int16 temperature_adc_x
    int16 temperature_adc_y
    int16 temperature_adc_z
    int16 temperature_adc_cpu

Топик: gkv_adc_data

---------------------------------------------
Стандартный пакет данных в режиме "Данные с датчиков":

Сообщение: GkvSensorsData.msg

Поля сообщения:

    std_msgs/Header header
    uint16 status
    uint16 sample_counter
    float32 acceleration_x
    float32 acceleration_y
    float32 acceleration_z
    float32 angular_rate_x
    float32 angular_rate_y
    float32 angular_rate_z
    float32 temperature_x
    float32 temperature_y
    float32 temperature_z
    float32 temperature_cpu

Топик: gkv_sensors_data

---------------------------------------------
Стандартный пакет данных в режимах "AHRS:Фильтр Калмана" и "AHRS:Фильтр Махони":
Сообщение: GkvGyrovertData.msg

Поля сообщения:

    std_msgs/Header header
    uint16 status
    uint16 sample_counter
    float32 pitch
    float32 roll
    float32 yaw

Топик: gkv_gyrovert_data

---------------------------------------------
Стандартный пакет данных в режиме "Инклинометр":
Сообщение: GkvInclinometerData.msg

Поля сообщения:

    std_msgs/Header header
    uint16 status
    uint16 sample_counter
    float32 alpha
    float32 beta

Топик: gkv_inclinometer_data

---------------------------------------------
Стандартный пакет данных в режимах "БИНС","БИНС:ESKF+СНС" и"БИНС:ESKF5+СНС":
Сообщение: GkvBINSData.msg

Поля сообщения:

    std_msgs/Header header
    uint16 status
    uint16 sample_counter
    float32 x
    float32 y
    float32 z
    float32 pitch
    float32 roll
    float32 yaw
    float32 alpha
    float32 beta
    float32 q3
    float32 q2
    float32 q1
    float32 q0

Топик: gkv_bins_data

---------------------------------------------
Стандартный пакет данных ГНСС во всех режимах:
Сообщение: GkvGpsData.msg

Поля сообщения:

    std_msgs/Header header
    float32 time
    float32 latitude
    float32 longitude
    float32 altitude
    uint32 gps_state_status
    float32 TDOP
    float32 HDOP
    float32 VDOP
    float32 horizontal_vel
    float32 azimuth
    float32 vertical_vel

Топик: gkv_gnss_data

---------------------------------------------
Расширенный пакет данных ГНСС во всех режимах:
Сообщение: GkvExtGpsData.msg

Поля сообщения:

    std_msgs/Header header
    float32 lat_vel
    float32 lon_vel
    float32 lat_std
    float32 lon_std
    float32 alt_std
    float32 lat_vel_std
    float32 lon_vel_std
    float32 alt_vel_std
    uint16 num_ss

Топик: gkv_ext_gnss_data

---------------------------------------------
Наборный пакет данных во всех режимах:
Сообщение: GkvCustomData.msg

Поля сообщения:

    std_msgs/Header header
    uint8 quantity_of_params
    uint8[] numbers
    float32[] param_values

Топик: gkv_custom_data


---------------------------------------------
При выборе режима GKV_ROS_PACKET_MODE вместо GKV_LMP_PACKET_MODE  устройство выдает сообщения в стандартном формате  сообщений ROS:

Сообщение:geometry_msgs/PoseStamped

Топик: gkv_pose_stamped_data


---------------------------------------------
Сервисы: 
 - проверка соединения
 - программный сброс
 - запрос ID устройства 
 - установка алгоритма работы 
 - установка типа пакета данных 
 - установка параметров наборного пакета
 осуществляются подключением клиента к соответствующему сервису:


Проверка соединения: gkv_check_connection_srv.

  Запрос: без заполнения полей.

  Ответ: результат, поле "result", значения "true/false"


Сброс: gkv_reset_srv.

  Запрос: без заполнения полей.

  Ответ: результат, поле "result", значения "true/false"


Установка алгоритма: gkv_set_alg_srv

  Запрос: номер алгоритма, поле "algorithm_number", значения:
  
    0 - Коды АЦП
    1 - Данные с датчиков
    2 - AHRS:Фильтр Калмана
    4 - Инклинометр
    5 - AHRS:Фильтр Махони
    6 - БИНС
    7 - БИНС:ESKF+СНС
    8 - Пользовательский
    9 - БИНС:ESKF5+СНС
  
  Ответ: результат, поле result, значения "true/false"


Установка типа пакета: gkv_set_packet_type_srv

  Запрос: тип пакета, поле "packet_type", значения:
  
    GKV_SET_DEFAULT_ALGORITHM_PACKET = 0 - Стандартный пакет для выбранного алгоритма
    GKV_SET_CUSTOM_PACKET = 1 - Наборный пакет данных
  
  Ответ: результат, поле result, значения "true/false"
  
  
Установка параметров наборного пакета: gkv_set_custom_params_srv

  Запрос: 
   - количество параметров, поле "quantity_of_params", значения: 0-63
   - номера параметров, массив uint8 "params", значения каждого элемента массива: 0-109 (перечень имён параметров указан в файле LMP_CustomPacket.h)
  
  Ответ: результат, поле result, значения "true/false"


Запрос ID: gkv_get_id_srv.

  Запрос: без заполнения полей. 

  Ответ: результат, поле "dev_description", значение - строка с типом устройства, поле "dev_id", значение - строка с серийными номером устройства
  

Пример
------------------
Пример работы библиотеки (находится в папке src) включает в себя четыре узла, связанные по всем перечисленым топикам. 

Принимающий узел (gkv_controller) считывает данные с выбранного пользователем serial-порта и передает их в топики, а также поднимает сервисы проверки соединения, сброса, установки алгоритма и запроса ID устройства. 

Принимающий узел (gkv_controller_ros_mode) считывает данные с выбранного пользователем serial-порта и устанавливает режим выдачи данных в формате стандартных ROS сообщений geometry_msgs/PoseStamped.

Обрабатывающий узел (gkv_listener) слушает все перечисленные топики и выводит принятые из них данные в консоль.

Обрабатывающий узел (gkv_alg_switcher) слушает все перечисленные топики и выводит принятые из них данные в консоль. При инициализации проверяет соединение, сбрасывает устройство, устанавливает алгоритм "Коды АЦП" и запрашивает ID.

Обрабатывающий узел (gkv_custom_packet_listener) слушает топик "gkv_custom_data" и выводит принятые из них данные в консоль. При инициализации проверяет соединение, устанавливает алгоритм "AHRS:Фильтр Калмана" и устанавливает ноборный пакет данных, содержащий ориентацию в углах эйлера и кватернион.
