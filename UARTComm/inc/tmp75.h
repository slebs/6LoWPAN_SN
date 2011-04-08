// talk to tmp75/175/275 family of i2c temperature sensors
// eric volpe (epvuc@limpoc.com) 3/18/2009

#define TMP75_OS 	(1<<7)
#define TMP75_12BIT	((1<<6)|(1<<5))
#define TMP75_11BIT	(1<<6)
#define TMP75_10BIT	(1<<5)
#define TMP75_9BIT	0
#define TMP75_FQ1	0
#define TMP75_FQ2	(1<<3)
#define TMP75_FQ4	(1<<4)
#define TMP75_FQ6	((1<<3)|(1<<4))
#define TMP75_POL	(1<<2)
#define TMP75_TM	(1<<1)
#define	TMP75_SD	(1<<0)

uint16_t tmp75_read_temp(void);
uint16_t tmp75_read_tlow(void);
uint16_t tmp75_read_thi(void);
uint8_t tmp75_read_config(void);
void tmp75_write_config(uint8_t conf);
void tmp75_write_tlow(uint16_t tlow);
void tmp75_write_thi(uint16_t thi);
