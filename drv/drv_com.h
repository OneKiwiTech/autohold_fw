#ifndef __DRV_COM_H
#define __DRV_COM_H

#include "fsl_common.h"
#include "fsl_iocon.h"

#define SET_BIT(REG, BIT)     ((REG) |= (BIT))
#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))
#define READ_BIT(REG, BIT)    ((REG) & (BIT))
#define CLEAR_REG(REG)        ((REG) = (0x0))
#define WRITE_REG(REG, VAL)   ((REG) = (VAL))
#define READ_REG(REG)         ((REG))
#define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))
#define POSITION_VAL(VAL)     (__CLZ(__RBIT(VAL))) 


#define IOCON_PIO_DIGITAL_EN        0x0100u /*!<@brief Enables digital function */
#define IOCON_PIO_FUNC1             0x01u /*!<@brief Selects pin function 1 */
#define IOCON_PIO_FUNC2             0x02u /*!<@brief Selects pin function 2 */
#define IOCON_PIO_FUNC7             0x07u /*!<@brief Selects pin function 7 */
#define IOCON_PIO_FUNC8             0x08u /*!<@brief Selects pin function 8 */
#define IOCON_PIO_INV_DI            0x00u /*!<@brief Input function is not inverted */
#define IOCON_PIO_MODE_PULLUP       0x20u /*!<@brief Selects pull-up function */
#define IOCON_PIO_OPENDRAIN_DI      0x00u /*!<@brief Open drain is disabled */
#define IOCON_PIO_SLEW_STANDARD     0x00u /*!<@brief Standard mode, output slew rate control is enabled */



#ifdef __cplusplus
 extern "C" {
#endif


#ifdef __cplusplus
}
#endif

#endif /* __DRV_COM_H */


/************************ (C) COPYRIGHT Kien Minh Co.,Ltd *****END OF FILE****/
