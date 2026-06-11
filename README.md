# drone_fc_dualcore

🚁 RP2040 Dual-Core Uçuş Kontrolcüsü (F450 Quadcopter)
Bu proje, F450 kasa quadcopterler için sıfırdan C++ ile geliştirilmiş, Raspberry Pi Pico (RP2040) tabanlı özel bir uçuş kontrol yazılımıdır. Piyasada bulunan hazır uçuş kontrolcülerinin (Pixhawk, Matek vb.) aksine, sistemin tüm uçuş matematiği, PID döngüleri ve sensör füzyonu algoritmaları bağımsız olarak inşa edilmiştir.

Projenin en büyük mühendislik odak noktası; uçuş matematiği ile telemetri haberleşmesini birbirinden tamamen izole eden Asimetrik Çoklu İşlem (AMP - Dual Core) mimarisidir.

🌟 Temel Özellikler
Asimetrik Çift Çekirdek (Dual-Core) Mimarisi:

Core 0 (Uçuş Beyni): Kesintisiz bir döngüde (loop) sadece MPU6050 okumaları, Cascade PID hesaplamaları, Motor/ESC sinyal güncellemeleri ve otonom iniş matematiğini çalıştırır. Asla bekleme (delay) veya yazdırma işlemi yapmaz. Sabit ve ultra düşük bir dt (Delta Time) garantiler.

Core 1 (İletişim & Telemetri): loop1 üzerinde çalışarak yer istasyonundan (GCS) gelen komutları işler ve Ring Buffer (Halka Tampon) mantığıyla HC-12 modülü üzerinden telemetri verilerini aktarır. Telsiz haberleşmesinden kaynaklanan Loop Blocking (Döngü Tıkanması) sorunu kökünden çözülmüştür.

Cascade (Şelale) PID Kontrolü: Roll ve Pitch eksenlerinde yüksek stabilite için Açı (Angle) ve Açısal Hız (Rate) olmak üzere iç içe geçmiş iki katmanlı PID algoritması kullanılmıştır.

3-Fazlı Otonom Güvenli İniş: VL53L0X LiDAR entegrasyonu sayesinde sistem irtifayı otonom olarak tarar. Belirli bir yüksekliğe kadar kör iniş, Lidar menziline girildiğinde kontrollü süzülme ve yere temas anında otomatik motor kesme (DISARM) işlemlerini gerçekleştirir.

Hızlı I2C Veri Yolu: MPU6050 sensörü ile olan iletişim, standart hız yerine 400 kHz (Fast Mode) seviyesine çekilerek sensör okuma gecikmeleri minimize edilmiştir.

🛠️ Kullanılan Donanımlar
Mikrodenetleyici: Raspberry Pi Pico (RP2040)

IMU (Ataletsel Ölçüm Ünitesi): MPU6050 (6 Eksenli Jiroskop ve İvmeölçer)

Mesafe Sensörü (İrtifa): VL53L0X Time-of-Flight LiDAR

Haberleşme: HC-12 433MHz RF Telsiz Modülü

ESC: XW-30A (Fırçasız Motor Sürücü)

Şase & Pervane: F450 Quadcopter Kasa, 10 inç (1045) Pervaneler

⚙️ Kurulum ve Derleme (Arduino IDE)
Bu yazılım Arduino IDE üzerinde Earle F. Philhower'ın hazırladığı RP2040 çekirdeği kullanılarak derlenmelidir.

Arduino IDE'de Araçlar > Kart > Boards Manager menüsüne gidin.

Raspberry Pi Pico/RP2040 (by Earle F. Philhower, III) kütüphanesini kurun.

Gerekli sensör kütüphanelerini yükleyin:

Wire.h (Dahili)

VL53L0X.h (Pololu)

Kodu derlerken Araçlar menüsünden Raspberry Pi Pico kartını seçtiğinizden emin olun. (Çift çekirdek özellikleri bu çekirdek ile otomatik aktif olur).

⚠️ Güvenlik ve Uçuş Öncesi Uyarılar
Pervaneleri Çıkarın: USB üzerinden kod yüklerken veya yer istasyonu ile masaüstü testleri yaparken pervanelerin SÖKÜLÜ olduğundan kesinlikle emin olun.

Güç Çakışması: Pico, bilgisayara USB ile bağlıyken drona LiPo batarya TAKMAYIN (Eğer VSYS pinine dışarıdan besleme yapıyorsanız donanım hasarı oluşabilir).

Sensör Kalibrasyonu: Sistem açıldığında MPU6050 otomatik dara/offset alır. Bataryayı takarken dronun tamamen düz bir zeminde ve hareketsiz (titreşimsiz) olmasına dikkat edin.

Motor Yönleri (Quad-X): Motor mikseri Quad-X konfigürasyonuna göre yazılmıştır. Pervanelerin saat yönü (CW) ve saat yönü tersi (CCW) takılış sırasını uçuş öncesi mutlaka kontrol edin.

🤝 Geliştirici Ekip
Bu proje, milli teknoloji üretme heyecanıyla sahada aktif olarak test edilmiş ve geliştirilmiştir.

Murat Emir Mamuş - Yazılım Mimarisi & Gömülü Sistemler

Cemil Mete Çonkara - Donanım & Test

Murat Fetih Kaya - Donanım & Test

Not: Bu sistem otonom uçuş algoritmaları geliştirmek, PID teorilerini fiziksel olarak test etmek ve uçuş kontrolcüleri mimarisini (Flight Controller Architecture) öğrenmek amacıyla yapılmış deneysel bir mühendislik projesidir.
