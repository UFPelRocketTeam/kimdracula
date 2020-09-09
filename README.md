#kimdracula v1 TODO: fazer manual bonitinho

Documentação de suporte:

https://www.st.com/content/ccc/resource/technical/document/errata_sheet/7f/05/b0/bc/34/2f/4c/21/CD00288116.pdf/files/CD00288116.pdf/jcr:content/translations/en.CD00288116.pdf Errata sobre a HAL do ST. A parte sobre o barramento I²C é aplicavel ao STM32F103.


Workaround: iniciar o CLK do barramento i2c antes de definir as GPIOs. 

Talvez o struct que ele usa pra criar as GPIOs do i2c não funcione out of the box. Usar:

#define I2C1_SCL_Pin GPIO_PIN_6
#define I2C1_SCL_GPIO_Port GPIOB
#define I2C1_SDA_Pin GPIO_PIN_7
#define I2C1_SDA_GPIO_Port GPIOB

Nota mental: não gerar arquivos de projeto direto do CubeMX e esperar que funcione. Usar a HAL desse projeto como base pros futuros, especialmente se usar I2C

Errata pertinente ao STM32F103xC:
https://www.st.com/content/ccc/resource/technical/document/errata_sheet/f5/50/c9/46/56/db/4a/f6/CD00197763.pdf/files/CD00197763.pdf/jcr:content/translations/en.CD00197763.pdf
