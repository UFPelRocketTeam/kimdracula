#Kimdracula V1

Placa de desenvolvimento, instrumentação inercial e telemetria. Baseada no STM32F103C8T6, usando o Si4463 como módulo de rádio e sensor fusion com o MPU6050 e HCM5883. Essa é a parte que vai ficar bonitinha depois, o resto to usando como devlog mesmo e treinando markdown

1. O STM32
  1. A HAL

1. O Si4463
  1. PATCH_CMDS

1. A IMU
  1. Portando pro STM32
  1. Usando a DMP


#Documentação de suporte:

[Errata sobre a HAL do ST. A parte sobre o barramento I²C é aplicavel ao STM32F103.](https://www.st.com/content/ccc/resource/technical/document/errata_sheet/7f/05/b0/bc/34/2f/4c/21/CD00288116.pdf/files/CD00288116.pdf/jcr:content/translations/en.CD00288116.pdf)

Workaround: iniciar o CLK do barramento i2c antes de definir as GPIOs. 

Talvez o struct que ele usa pra criar as GPIOs do i2c não funcione out of the box. Usar:
```C
#define I2C1_SCL_Pin GPIO_PIN_6
#define I2C1_SCL_GPIO_Port GPIOB
#define I2C1_SDA_Pin GPIO_PIN_7
#define I2C1_SDA_GPIO_Port GPIOB
```


Nota mental: ***não*** gerar arquivos de projeto direto do CubeMX e esperar que funcione. Usar a HAL desse projeto como base pros futuros, especialmente se usar I2C. Até porque a parte do debug tá funcionando aqui e eu não lembro o que eu precisei fazer pra habilitar o debug pelo st-link


Aparentemente a ST programou essa HAL com a bunda. [Errata pertinente ao STM32F103xC:](https://www.st.com/content/ccc/resource/technical/document/errata_sheet/f5/50/c9/46/56/db/4a/f6/CD00197763.pdf/files/CD00197763.pdf/jcr:content/translations/en.CD00197763.pdf)

Catando aquele tutorial maroto do dito ChibiOS pra ver se rola sentimento

update: chibios nada eras

vamo bate em frente pela hal até segunda ordem
