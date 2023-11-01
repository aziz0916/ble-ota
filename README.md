# ESP32-H2 專案

使用 ESP32-H2-DevKitM-1 開發板實現 BLE、OTA、Light Sleep 與 SPIFFS 等功能

## BLE

使用 gatt_server_service_table 範例實現 BLE 與新增 service

## OTA

採用 Siliconlabs 的 efr connect app 支持 ota 的功能

### OTA 實現方法

- 手機 app Siliconlabs efr connect(efr connect ota 只支持傳輸.gbl 的檔案形式，因此待更新的 bin 檔案直接將.bin 後綴改為.gbl 即可)
- 使用 Siliconlabs 的 efr connect 來做 ota client，首先需要實現 Siliconlabs 自定義的 service 以及 char，該部分定義可以從 Siliconlabs 的開發工具獲得相關信息，Siliconlabs 的 ota 相關的 service 以及 char 定義如下:
  1、
  Service Name: Silicon Labs OTA
  Type: com.silabs.service.ota
  UUID: 1D14D6EE-FD63-4FA1-BFA4-8F47B42119F0
  Source: Silicon Labs
  Abstract: The Silicon Labs OTA Service enables over-the-air firmware update of the device.

  2、
  Characteristics of the service: Silicon Labs OTA Control
  Characteristic Name: Silicon Labs OTA Control
  Type: com.silabs.characteristic.ota_control
  UUID: F7BF3564-FB6D-4E53-88A4-5E37E0326063
  Source: Silicon Labs
  Abstract: Silicon Labs OTA Control.

      Property requirements:
       Reliable write - Excluded
       Indicate - Excluded
       Write Without Response - Excluded
       Read - Excluded
       Write - Mandatory
       Notify - Excluded

  3、
  Silicon Labs OTA Data Characteristic
  Name: Silicon Labs OTA Data
  Type: com.silabs.characteristic.ota_data
  UUID: 984227F3-34FC-4045-A5D0-2C581F81A153
  Source: Silicon Labs
  Abstract: Silicon Labs OTA Data.

       Property requirements:
        Reliable write - Excluded
        Indicate - Excluded
        Write Without Response - Mandatory
        Read - Excluded
        Write - Mandatory
        Notify - Excluded

- Siliconlabs efr connect app 通過如上 char 進行 ota 流程，如下:

 - efr connect 在開始 ota 的時候會對 ota control 寫 0，以提示設備準備開始 ota，設備可以在此階段進行 erase flash
 - 透過 ota data char 分段傳輸更新的檔案，直至檔案傳輸完成
 - 檔案傳輸完成後，efr connect 會透過對 ota control 寫 3，已提示 ota 檔案傳輸完成，此時設備可以準備開始啟動新檔案

- efr connect app 的 ota 操作過程
  ![图片](https://user-images.githubusercontent.com/30143031/132782483-cf12eb56-f63d-42b5-a9f1-b7cea81b0d34.png)

## Light Sleep

使用 light_sleep 範例實現

### Light Sleep 實現方法

- 系統一啟動會先進入 light sleep
- 使用 ESP32-H2-DevKitM-1 開發板上 Boot Button 下緣觸發啟動藍芽廣播，待藍芽斷線後會重新啟動以進入 light sleep
- 當溫度小於等於 1°C 時，GPIO 10 上緣觸發會離開 light sleep，待工作完成後會重新啟動以進入 light sleep
- 當溫度小於等於-1°C 時，GPIO 11 上緣觸發會離開 light sleep，待工作完成後會重新啟動以進入 light sleep
- 當溫度小於等於-5°C 時，GPIO 12 上緣觸發會離開 light sleep，待工作完成後會重新啟動以進入 light sleep
- 當設備低電量時，GPIO 13 上緣觸發會離開 light sleep，待工作完成後會重新啟動以進入 light sleep
- 當設備異常時，GPIO 14 上緣觸發會離開 light sleep，待工作完成後會重新啟動以進入 light sleep

## SPIFFS

使用 spiffs 範例實現

### SPIFFS 實現方法

- 當 GPIO 10~14 上緣觸發時會進行 spiffs 資料寫入程序，如下:

 - 對 spiffs 進行配置
 - 將 spiffs 掛載與註冊到 vfs(虛擬文件系統)
 - 查看 spiffs 訊息
 - 新增或開啟'data.json'檔案，並寫入 json 格式的事件資料
 - 開啟'data.json'檔案，讀取所有事件資料
 - 將 spiffs 從 vfs(虛擬文件系統)卸載與取消註冊

- 事件資料格式如下:

```
'{
    "event":        "Temperature1 ≤ 1°C",'
    "time after startup(ms)":       9350'
}'
```
