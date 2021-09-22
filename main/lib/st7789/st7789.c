#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <driver/spi_master.h>
#include <driver/gpio.h>
#include "esp_log.h"

#include "st7789.h"

#define TAG "ST7789"
#define	_DEBUG_ 0

#ifdef CONFIG_IDF_TARGET_ESP32
#define LCD_HOST    HSPI_HOST
#define DMA_CHAN    2
#elif defined CONFIG_IDF_TARGET_ESP32S2
#define LCD_HOST    SPI2_HOST
#define DMA_CHAN    LCD_HOST
#endif

#define DEG2RAD 0.0174532925
#ifndef _swap_int16_t
#define _swap_int16_t(a, b)                                                    \
  {                                                                            \
    int16_t t = a;                                                             \
    a = b;                                                                     \
    b = t;                                                                     \
  }
#endif

//static const int GPIO_MOSI = 23;
//static const int GPIO_SCLK = 18;

static const int SPI_Command_Mode = 0;
static const int SPI_Data_Mode = 1;
//static const int SPI_Frequency = SPI_MASTER_FREQ_20M;
//static const int SPI_Frequency = SPI_MASTER_FREQ_26M;
static const int SPI_Frequency = SPI_MASTER_FREQ_40M;
//static const int SPI_Frequency = SPI_MASTER_FREQ_80M;

static uint16_t _frame_buffer[240*240] = {BLACK};

void spi_master_init(TFT_t * dev, int16_t GPIO_MOSI, int16_t GPIO_SCLK, int16_t GPIO_CS, int16_t GPIO_DC, int16_t GPIO_RESET, int16_t GPIO_BL)
{
	esp_err_t ret;

	ESP_LOGI(TAG, "GPIO_CS=%d",GPIO_CS);
	if ( GPIO_CS >= 0 ) {
		gpio_pad_select_gpio( GPIO_CS );
		gpio_set_direction( GPIO_CS, GPIO_MODE_OUTPUT );
		gpio_set_level( GPIO_CS, 0 );
	}

	ESP_LOGI(TAG, "GPIO_DC=%d",GPIO_DC);
	gpio_pad_select_gpio( GPIO_DC );
	gpio_set_direction( GPIO_DC, GPIO_MODE_OUTPUT );
	gpio_set_level( GPIO_DC, 0 );

	ESP_LOGI(TAG, "GPIO_RESET=%d",GPIO_RESET);
	if ( GPIO_RESET >= 0 ) {
		gpio_pad_select_gpio( GPIO_RESET );
		gpio_set_direction( GPIO_RESET, GPIO_MODE_OUTPUT );
		gpio_set_level( GPIO_RESET, 1 );
		delayMS(50);
		gpio_set_level( GPIO_RESET, 0 );
		delayMS(50);
		gpio_set_level( GPIO_RESET, 1 );
		delayMS(50);
	}

	ESP_LOGI(TAG, "GPIO_BL=%d",GPIO_BL);
	if ( GPIO_BL >= 0 ) {
		gpio_pad_select_gpio(GPIO_BL);
		gpio_set_direction( GPIO_BL, GPIO_MODE_OUTPUT );
		gpio_set_level( GPIO_BL, 0 );
	}

	ESP_LOGI(TAG, "GPIO_MOSI=%d",GPIO_MOSI);
	ESP_LOGI(TAG, "GPIO_SCLK=%d",GPIO_SCLK);
	spi_bus_config_t buscfg = {
		.sclk_io_num = GPIO_SCLK,
		.mosi_io_num = GPIO_MOSI,
		.miso_io_num = -1,
		.quadwp_io_num = -1,
		.quadhd_io_num = -1
	};

	ret = spi_bus_initialize( LCD_HOST, &buscfg, DMA_CHAN );
	ESP_LOGD(TAG, "spi_bus_initialize=%d",ret);
	assert(ret==ESP_OK);

	spi_device_interface_config_t devcfg={
		.clock_speed_hz = SPI_Frequency,
		.queue_size = 7,
		.mode = 2,
		.flags = SPI_DEVICE_NO_DUMMY,
	};

	if ( GPIO_CS >= 0 ) {
		devcfg.spics_io_num = GPIO_CS;
	} else {
		devcfg.spics_io_num = -1;
	}
	
	spi_device_handle_t handle;
	ret = spi_bus_add_device( LCD_HOST, &devcfg, &handle);
	ESP_LOGD(TAG, "spi_bus_add_device=%d",ret);
	assert(ret==ESP_OK);
	dev->_dc = GPIO_DC;
	dev->_bl = GPIO_BL;
	dev->_SPIHandle = handle;
}


bool spi_master_write_byte(spi_device_handle_t SPIHandle, const uint8_t* Data, size_t DataLength)
{
	spi_transaction_t SPITransaction;
	esp_err_t ret;

	if ( DataLength > 0 ) {
		memset( &SPITransaction, 0, sizeof( spi_transaction_t ) );
		SPITransaction.length = DataLength * 8;
		SPITransaction.tx_buffer = Data;
#if 1
		ret = spi_device_transmit( SPIHandle, &SPITransaction );
#endif
#if 0
		ret = spi_device_polling_transmit( SPIHandle, &SPITransaction );
#endif
		assert(ret==ESP_OK); 
	}

	return true;
}

bool spi_master_write_command(TFT_t * dev, uint8_t cmd)
{
	static uint8_t Byte = 0;
	Byte = cmd;
	gpio_set_level( dev->_dc, SPI_Command_Mode );
	return spi_master_write_byte( dev->_SPIHandle, &Byte, 1 );
}

bool spi_master_write_data_byte(TFT_t * dev, uint8_t data)
{
	static uint8_t Byte = 0;
	Byte = data;
	gpio_set_level( dev->_dc, SPI_Data_Mode );
	return spi_master_write_byte( dev->_SPIHandle, &Byte, 1 );
}


bool spi_master_write_data_word(TFT_t * dev, uint16_t data)
{
	static uint8_t Byte[2];
	Byte[0] = (data >> 8) & 0xFF;
	Byte[1] = data & 0xFF;
	gpio_set_level( dev->_dc, SPI_Data_Mode );
	return spi_master_write_byte( dev->_SPIHandle, Byte, 2);
}

bool spi_master_write_addr(TFT_t * dev, uint16_t addr1, uint16_t addr2)
{
	static uint8_t Byte[4];
	Byte[0] = (addr1 >> 8) & 0xFF;
	Byte[1] = addr1 & 0xFF;
	Byte[2] = (addr2 >> 8) & 0xFF;
	Byte[3] = addr2 & 0xFF;
	gpio_set_level( dev->_dc, SPI_Data_Mode );
	return spi_master_write_byte( dev->_SPIHandle, Byte, 4);
}

bool spi_master_write_color(TFT_t * dev, uint16_t color, uint16_t size)
{
	static uint8_t Byte[1024];
	int index = 0;
	for(int i=0;i<size;i++) {
		Byte[index++] = (color >> 8) & 0xFF;
		Byte[index++] = color & 0xFF;
	}
	gpio_set_level( dev->_dc, SPI_Data_Mode );
	return spi_master_write_byte( dev->_SPIHandle, Byte, size*2);
}

// Add 202001
bool spi_master_write_colors(TFT_t * dev, uint16_t * colors, uint16_t size)
{
	static uint8_t Byte[1024];
	int index = 0;
	for(int i=0;i<size;i++) {
		Byte[index++] = (colors[i] >> 8) & 0xFF;
		Byte[index++] = colors[i] & 0xFF;
	}
	gpio_set_level( dev->_dc, SPI_Data_Mode );
	return spi_master_write_byte( dev->_SPIHandle, Byte, size*2);
}

void delayMS(int ms) {
	int _ms = ms + (portTICK_PERIOD_MS - 1);
	TickType_t xTicksToDelay = _ms / portTICK_PERIOD_MS;
	ESP_LOGD(TAG, "ms=%d _ms=%d portTICK_PERIOD_MS=%d xTicksToDelay=%d",ms,_ms,portTICK_PERIOD_MS,xTicksToDelay);
	vTaskDelay(xTicksToDelay);
}


void lcdInit(TFT_t * dev, int width, int height, int offsetx, int offsety)
{
	dev->_width = width;
	dev->_height = height;
	dev->_offsetx = offsetx;
	dev->_offsety = offsety;
	dev->_font_direction = DIRECTION0;
	dev->_font_fill = false;
	dev->_font_underline = false;

	spi_master_write_command(dev, 0x01);	//Software Reset
	delayMS(150);

	spi_master_write_command(dev, 0x11);	//Sleep Out
	delayMS(255);
	
	spi_master_write_command(dev, 0x3A);	//Interface Pixel Format
	spi_master_write_data_byte(dev, 0x55);
	delayMS(10);
	
	spi_master_write_command(dev, 0x36);	//Memory Data Access Control
	spi_master_write_data_byte(dev, 0x00);

	spi_master_write_command(dev, 0x2A);	//Column Address Set
	spi_master_write_data_byte(dev, 0x00);
	spi_master_write_data_byte(dev, 0x00);
	spi_master_write_data_byte(dev, 0x00);
	spi_master_write_data_byte(dev, 0xF0);

	spi_master_write_command(dev, 0x2B);	//Row Address Set
	spi_master_write_data_byte(dev, 0x00);
	spi_master_write_data_byte(dev, 0x00);
	spi_master_write_data_byte(dev, 0x00);
	spi_master_write_data_byte(dev, 0xF0);

	spi_master_write_command(dev, 0x21);	//Display Inversion On
	delayMS(10);

	spi_master_write_command(dev, 0x13);	//Normal Display Mode On
	delayMS(10);

	lcdFillScreen(dev, BLACK);				//Write full screen of black
	lcdUpdate(dev);

	spi_master_write_command(dev, 0x29);	//Display ON
	delayMS(255);

	if(dev->_bl >= 0) {
		gpio_set_level( dev->_bl, 1 );
	}
}


// Draw pixel
// x:X coordinate
// y:Y coordinate
// color:color
void lcdDrawPixel(TFT_t * dev, uint16_t x, uint16_t y, uint16_t color){
	if (x >= dev->_width) return;
	if (y >= dev->_height) return;

	uint16_t _x = x + dev->_offsetx;
	uint16_t _y = y + dev->_offsety;

	_frame_buffer[_x + (_y * 240)] = color; // Update frame buffer at pixel location
}


// Draw multi pixel
// x:X coordinate
// y:Y coordinate
// size:Number of colors
// colors:colors
void lcdDrawMultiPixels(TFT_t * dev, uint16_t x, uint16_t y, uint16_t size, uint16_t * colors) {
	if (x+size > dev->_width) return;
	if (y >= dev->_height) return;

	uint16_t _x1 = x + dev->_offsetx;
	uint16_t _y1 = y + dev->_offsety;

	//TODO: Fill memory faster
	for (int i=0; i<size; ++i) {
		_frame_buffer[_x1 + (_y1 * 240) + i] = colors[i];
	}
}

// Draw rectangle of filling
// x1:Start X coordinate
// y1:Start Y coordinate
// x2:End X coordinate
// y2:End Y coordinate
// color:color
void lcdDrawFillRect(TFT_t * dev, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color) {
	//if (x1 >= dev->_width) return;
	//if (x2 >= dev->_width) x2=dev->_width-1;
	//if (y1 >= dev->_height) return;
	//if (y2 >= dev->_height) y2=dev->_height-1;

	ESP_LOGD(TAG,"offset(x)=%d offset(y)=%d",dev->_offsetx,dev->_offsety);
	uint16_t _x1 = x1 + dev->_offsetx;
	uint16_t _x2 = x2 + dev->_offsetx;
	uint16_t _y1 = y1 + dev->_offsety;
	uint16_t _y2 = y2 + dev->_offsety;
	uint16_t size = _y2 -_y1 + 1;

	for(int i=_x1;i<=_x2;i++){
		//TODO: Fill memory faster
		for (int j=_y1; j<_y2+1; ++j) {
			for (int k=0; k<size; ++k) {
				_frame_buffer[i + (j * 240) + k] = color;
			}
		}
	}
}

// Display OFF
void lcdDisplayOff(TFT_t * dev) {
	spi_master_write_command(dev, 0x28);	//Display off
}
 
// Display ON
void lcdDisplayOn(TFT_t * dev) {
	spi_master_write_command(dev, 0x29);	//Display on
}

// Fill screen
// color:color
void lcdFillScreen(TFT_t * dev, uint16_t color) {
	//TODO: Fill memory faster
	for (int i=0; i<(240*240); ++i) {
		_frame_buffer[i] = color;
	}
}

// Draw line
// x1:Start X coordinate
// y1:Start Y coordinate
// x2:End X coordinate
// y2:End Y coordinate
// color:color 
void lcdDrawLine(TFT_t * dev, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color) {
	int i;
	int dx,dy;
	int sx,sy;
	int E;

	/* distance between two points */
	dx = ( x2 > x1 ) ? x2 - x1 : x1 - x2;
	dy = ( y2 > y1 ) ? y2 - y1 : y1 - y2;

	/* direction of two point */
	sx = ( x2 > x1 ) ? 1 : -1;
	sy = ( y2 > y1 ) ? 1 : -1;

	/* inclination < 1 */
	if ( dx > dy ) {
		E = -dx;
		for ( i = 0 ; i <= dx ; i++ ) {
			lcdDrawPixel(dev, x1, y1, color);
			x1 += sx;
			E += 2 * dy;
			if ( E >= 0 ) {
			y1 += sy;
			E -= 2 * dx;
		}
	}

	/* inclination >= 1 */
	} else {
		E = -dy;
		for ( i = 0 ; i <= dy ; i++ ) {
			lcdDrawPixel(dev, x1, y1, color);
			y1 += sy;
			E += 2 * dx;
			if ( E >= 0 ) {
				x1 += sx;
				E -= 2 * dy;
			}
		}
	}
}

// Draw rectangle
// x1:Start X coordinate
// y1:Start Y coordinate
// x2:End	X coordinate
// y2:End	Y coordinate
// color:color
void lcdDrawRect(TFT_t * dev, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color) {
	lcdDrawLine(dev, x1, y1, x2, y1, color);
	lcdDrawLine(dev, x2, y1, x2, y2, color);
	lcdDrawLine(dev, x2, y2, x1, y2, color);
	lcdDrawLine(dev, x1, y2, x1, y1, color);
}

// Draw rectangle with angle
// xc:Center X coordinate
// yc:Center Y coordinate
// w:Width of rectangle
// h:Height of rectangle
// angle :Angle of rectangle
// color :color

//When the origin is (0, 0), the point (x1, y1) after rotating the point (x, y) by the angle is obtained by the following calculation.
// x1 = x * cos(angle) - y * sin(angle)
// y1 = x * sin(angle) + y * cos(angle)
void lcdDrawRectAngle(TFT_t * dev, uint16_t xc, uint16_t yc, uint16_t w, uint16_t h, uint16_t angle, uint16_t color) {
	double xd,yd,rd;
	int x1,y1;
	int x2,y2;
	int x3,y3;
	int x4,y4;
	rd = -angle * M_PI / 180.0;
	xd = 0.0 - w/2;
	yd = h/2;
	x1 = (int)(xd * cos(rd) - yd * sin(rd) + xc);
	y1 = (int)(xd * sin(rd) + yd * cos(rd) + yc);

	yd = 0.0 - yd;
	x2 = (int)(xd * cos(rd) - yd * sin(rd) + xc);
	y2 = (int)(xd * sin(rd) + yd * cos(rd) + yc);

	xd = w/2;
	yd = h/2;
	x3 = (int)(xd * cos(rd) - yd * sin(rd) + xc);
	y3 = (int)(xd * sin(rd) + yd * cos(rd) + yc);

	yd = 0.0 - yd;
	x4 = (int)(xd * cos(rd) - yd * sin(rd) + xc);
	y4 = (int)(xd * sin(rd) + yd * cos(rd) + yc);

	lcdDrawLine(dev, x1, y1, x2, y2, color);
	lcdDrawLine(dev, x1, y1, x3, y3, color);
	lcdDrawLine(dev, x2, y2, x4, y4, color);
	lcdDrawLine(dev, x3, y3, x4, y4, color);
}

// Draw triangle
// xc:Center X coordinate
// yc:Center Y coordinate
// w:Width of triangle
// h:Height of triangle
// angle :Angle of triangle
// color :color

//When the origin is (0, 0), the point (x1, y1) after rotating the point (x, y) by the angle is obtained by the following calculation.
// x1 = x * cos(angle) - y * sin(angle)
// y1 = x * sin(angle) + y * cos(angle)
void lcdDrawTriangle(TFT_t * dev, uint16_t xc, uint16_t yc, uint16_t w, uint16_t h, uint16_t angle, uint16_t color) {
	double xd,yd,rd;
	int x1,y1;
	int x2,y2;
	int x3,y3;
	rd = -angle * M_PI / 180.0;
	xd = 0.0;
	yd = h/2;
	x1 = (int)(xd * cos(rd) - yd * sin(rd) + xc);
	y1 = (int)(xd * sin(rd) + yd * cos(rd) + yc);

	xd = w/2;
	yd = 0.0 - yd;
	x2 = (int)(xd * cos(rd) - yd * sin(rd) + xc);
	y2 = (int)(xd * sin(rd) + yd * cos(rd) + yc);

	xd = 0.0 - w/2;
	x3 = (int)(xd * cos(rd) - yd * sin(rd) + xc);
	y3 = (int)(xd * sin(rd) + yd * cos(rd) + yc);

	lcdDrawLine(dev, x1, y1, x2, y2, color);
	lcdDrawLine(dev, x1, y1, x3, y3, color);
	lcdDrawLine(dev, x2, y2, x3, y3, color);
}

/**************************************************************************/
/*!
   @brief     Draw a triangle with color-fill
    @param    x0  Vertex #0 x coordinate
    @param    y0  Vertex #0 y coordinate
    @param    x1  Vertex #1 x coordinate
    @param    y1  Vertex #1 y coordinate
    @param    x2  Vertex #2 x coordinate
    @param    y2  Vertex #2 y coordinate
    @param    color 16-bit 5-6-5 Color to fill/draw with
*/
/**************************************************************************/
void lcdFillTriangle(TFT_t * dev, int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color)
{
	int16_t a, b, y, last;

	// Sort coordinates by Y order (y2 >= y1 >= y0)
	if (y0 > y1) {
		_swap_int16_t(y0, y1);
		_swap_int16_t(x0, x1);
	}
	if (y1 > y2) {
		_swap_int16_t(y2, y1);
		_swap_int16_t(x2, x1);
	}
	if (y0 > y1) {
		_swap_int16_t(y0, y1);
		_swap_int16_t(x0, x1);
	}

	if (y0 == y2) { // Handle awkward all-on-same-line case as its own thing
		a = b = x0;
		if (x1 < a)
			a = x1;
		else if (x1 > b)
			b = x1;
		if (x2 < a)
			a = x2;
		else if (x2 > b)
			b = x2;

		lcdDrawLine(dev, a, y0, a + (b - a + 1) - 1, y0, color);
		return;
	}

	int16_t dx01 = x1 - x0, dy01 = y1 - y0, dx02 = x2 - x0, dy02 = y2 - y0,
	dx12 = x2 - x1, dy12 = y2 - y1;
	int32_t sa = 0, sb = 0;

	// For upper part of triangle, find scanline crossings for segments
	// 0-1 and 0-2.  If y1=y2 (flat-bottomed triangle), the scanline y1
	// is included here (and second loop will be skipped, avoiding a /0
	// error there), otherwise scanline y1 is skipped here and handled
	// in the second loop...which also avoids a /0 error here if y0=y1
	// (flat-topped triangle).
	if (y1 == y2)
		last = y1; // Include y1 scanline
	else
		last = y1 - 1; // Skip it

	for (y = y0; y <= last; y++) {
		a = x0 + sa / dy01;
		b = x0 + sb / dy02;
		sa += dx01;
		sb += dx02;
		/* longhand:
		a = x0 + (x1 - x0) * (y - y0) / (y1 - y0);
		b = x0 + (x2 - x0) * (y - y0) / (y2 - y0);
		*/
		if (a > b)
			_swap_int16_t(a, b);
		lcdDrawLine(dev, a, y, a + (b - a + 1) - 1, y, color);
	}

	// For lower part of triangle, find scanline crossings for segments
	// 0-2 and 1-2.  This loop is skipped if y1=y2.
	sa = (int32_t)dx12 * (y - y1);
	sb = (int32_t)dx02 * (y - y0);
	for (; y <= y2; y++) {
		a = x1 + sa / dy12;
		b = x0 + sb / dy02;
		sa += dx12;
		sb += dx02;
		/* longhand:
		a = x1 + (x2 - x1) * (y - y1) / (y2 - y1);
		b = x0 + (x2 - x0) * (y - y0) / (y2 - y0);
		*/
		if (a > b)
			_swap_int16_t(a, b);
		lcdDrawLine(dev, a, y, a + (b - a + 1) - 1, y, color);
	}
}

// Draw circle
// x0:Central X coordinate
// y0:Central Y coordinate
// r:radius
// color:color
void lcdDrawCircle(TFT_t * dev, uint16_t x0, uint16_t y0, uint16_t r, uint16_t color) {
	int x;
	int y;
	int err;
	int old_err;

	x=0;
	y=-r;
	err=2-2*r;
	do{
		lcdDrawPixel(dev, x0-x, y0+y, color); 
		lcdDrawPixel(dev, x0-y, y0-x, color); 
		lcdDrawPixel(dev, x0+x, y0-y, color); 
		lcdDrawPixel(dev, x0+y, y0+x, color); 
		if ((old_err=err)<=x)	err+=++x*2+1;
		if (old_err>y || err>x) err+=++y*2+1;	 
	} while(y<0);
}

// Draw circle of filling
// x0:Central X coordinate
// y0:Central Y coordinate
// r:radius
// color:color
void lcdDrawFillCircle(TFT_t * dev, uint16_t x0, uint16_t y0, uint16_t r, uint16_t color) {
	int x;
	int y;
	int err;
	int old_err;
	int ChangeX;

	x=0;
	y=-r;
	err=2-2*r;
	ChangeX=1;
	do{
		if(ChangeX) {
			lcdDrawLine(dev, x0-x, y0-y, x0-x, y0+y, color);
			lcdDrawLine(dev, x0+x, y0-y, x0+x, y0+y, color);
		} // endif
		ChangeX=(old_err=err)<=x;
		if (ChangeX)			err+=++x*2+1;
		if (old_err>y || err>x) err+=++y*2+1;
	} while(y<=0);
} 

// Draw rectangle with round corner
// x1:Start X coordinate
// y1:Start Y coordinate
// x2:End	X coordinate
// y2:End	Y coordinate
// r:radius
// color:color
void lcdDrawRoundRect(TFT_t * dev, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t r, uint16_t color) {
	int x;
	int y;
	int err;
	int old_err;
	unsigned char temp;

	if(x1>x2) {
		temp=x1; x1=x2; x2=temp;
	} // endif
	  
	if(y1>y2) {
		temp=y1; y1=y2; y2=temp;
	} // endif

	ESP_LOGD(TAG, "x1=%d x2=%d delta=%d r=%d",x1, x2, x2-x1, r);
	ESP_LOGD(TAG, "y1=%d y2=%d delta=%d r=%d",y1, y2, y2-y1, r);
	if (x2-x1 < r) return; // Add 20190517
	if (y2-y1 < r) return; // Add 20190517

	x=0;
	y=-r;
	err=2-2*r;

	do{
		if(x) {
			lcdDrawPixel(dev, x1+r-x, y1+r+y, color); 
			lcdDrawPixel(dev, x2-r+x, y1+r+y, color); 
			lcdDrawPixel(dev, x1+r-x, y2-r-y, color); 
			lcdDrawPixel(dev, x2-r+x, y2-r-y, color);
		} // endif 
		if ((old_err=err)<=x)	err+=++x*2+1;
		if (old_err>y || err>x) err+=++y*2+1;	 
	} while(y<0);

	ESP_LOGD(TAG, "x1+r=%d x2-r=%d",x1+r, x2-r);
	lcdDrawLine(dev, x1+r,y1  ,x2-r,y1	,color);
	lcdDrawLine(dev, x1+r,y2  ,x2-r,y2	,color);
	ESP_LOGD(TAG, "y1+r=%d y2-r=%d",y1+r, y2-r);
	lcdDrawLine(dev, x1  ,y1+r,x1  ,y2-r,color);
	lcdDrawLine(dev, x2  ,y1+r,x2  ,y2-r,color);  
} 

// Draw arrow
// x1:Start X coordinate
// y1:Start Y coordinate
// x2:End	X coordinate
// y2:End	Y coordinate
// w:Width of the botom
// color:color
// Thanks http://k-hiura.cocolog-nifty.com/blog/2010/11/post-2a62.html
void lcdDrawArrow(TFT_t * dev, uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1,uint16_t w,uint16_t color) {
	double Vx= x1 - x0;
	double Vy= y1 - y0;
	double v = sqrt(Vx*Vx+Vy*Vy);
	//	 printf("v=%f\n",v);
	double Ux= Vx/v;
	double Uy= Vy/v;

	uint16_t L[2],R[2];
	L[0]= x1 - Uy*w - Ux*v;
	L[1]= y1 + Ux*w - Uy*v;
	R[0]= x1 + Uy*w - Ux*v;
	R[1]= y1 - Ux*w - Uy*v;
	//printf("L=%d-%d R=%d-%d\n",L[0],L[1],R[0],R[1]);

	//lcdDrawLine(x0,y0,x1,y1,color);
	lcdDrawLine(dev, x1, y1, L[0], L[1], color);
	lcdDrawLine(dev, x1, y1, R[0], R[1], color);
	lcdDrawLine(dev, L[0], L[1], R[0], R[1], color);
}


// Draw arrow of filling
// x1:Start X coordinate
// y1:Start Y coordinate
// x2:End	X coordinate
// y2:End	Y coordinate
// w:Width of the botom
// color:color
void lcdDrawFillArrow(TFT_t * dev, uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1,uint16_t w,uint16_t color) {
	double Vx= x1 - x0;
	double Vy= y1 - y0;
	double v = sqrt(Vx*Vx+Vy*Vy);
	//printf("v=%f\n",v);
	double Ux= Vx/v;
	double Uy= Vy/v;

	uint16_t L[2],R[2];
	L[0]= x1 - Uy*w - Ux*v;
	L[1]= y1 + Ux*w - Uy*v;
	R[0]= x1 + Uy*w - Ux*v;
	R[1]= y1 - Ux*w - Uy*v;
	//printf("L=%d-%d R=%d-%d\n",L[0],L[1],R[0],R[1]);

	lcdDrawLine(dev, x0, y0, x1, y1, color);
	lcdDrawLine(dev, x1, y1, L[0], L[1], color);
	lcdDrawLine(dev, x1, y1, R[0], R[1], color);
	lcdDrawLine(dev, L[0], L[1], R[0], R[1], color);

	int ww;
	for(ww=w-1;ww>0;ww--) {
		L[0]= x1 - Uy*ww - Ux*v;
		L[1]= y1 + Ux*ww - Uy*v;
		R[0]= x1 + Uy*ww - Ux*v;
		R[1]= y1 - Ux*ww - Uy*v;
		//printf("Fill>L=%d-%d R=%d-%d\n",L[0],L[1],R[0],R[1]);
		lcdDrawLine(dev, x1, y1, L[0], L[1], color);
		lcdDrawLine(dev, x1, y1, R[0], R[1], color);
	}
}


// RGB565 conversion
// RGB565 is R(5)+G(6)+B(5)=16bit color format.
// Bit image "RRRRRGGGGGGBBBBB"
uint16_t rgb565_conv(uint16_t r,uint16_t g,uint16_t b) {
	return (((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}

static const uint8_t font1[] = {
  0x00, 0x00, 0x00, 0x00, 0x00,      // Code for char
  0x00, 0x06, 0x5F, 0x06, 0x00,      // Code for char !
  0x07, 0x03, 0x00, 0x07, 0x03,      // Code for char "
  0x24, 0x7E, 0x24, 0x7E, 0x24,      // Code for char #
  0x24, 0x2B, 0x6A, 0x12, 0x00,      // Code for char $
  0x63, 0x13, 0x08, 0x64, 0x63,      // Code for char %
  0x36, 0x49, 0x56, 0x20, 0x50,      // Code for char &
  0x00, 0x07, 0x03, 0x00, 0x00,      // Code for char '
  0x00, 0x3E, 0x41, 0x00, 0x00,      // Code for char (
  0x00, 0x41, 0x3E, 0x00, 0x00,      // Code for char )
  0x08, 0x3E, 0x1C, 0x3E, 0x08,      // Code for char *
  0x08, 0x08, 0x3E, 0x08, 0x08,      // Code for char +
  0x00, 0xE0, 0x60, 0x00, 0x00,      // Code for char ,
  0x08, 0x08, 0x08, 0x08, 0x08,      // Code for char -
  0x00, 0x60, 0x60, 0x00, 0x00,      // Code for char .
  0x20, 0x10, 0x08, 0x04, 0x02,      // Code for char /
  0x3E, 0x51, 0x49, 0x45, 0x3E,      // Code for char 0
  0x00, 0x42, 0x7F, 0x40, 0x00,      // Code for char 1
  0x62, 0x51, 0x49, 0x49, 0x46,      // Code for char 2
  0x22, 0x49, 0x49, 0x49, 0x36,      // Code for char 3
  0x18, 0x14, 0x12, 0x7F, 0x10,      // Code for char 4
  0x2F, 0x49, 0x49, 0x49, 0x31,      // Code for char 5
  0x3C, 0x4A, 0x49, 0x49, 0x30,      // Code for char 6
  0x01, 0x71, 0x09, 0x05, 0x03,      // Code for char 7
  0x36, 0x49, 0x49, 0x49, 0x36,      // Code for char 8
  0x06, 0x49, 0x49, 0x29, 0x1E,      // Code for char 9
  0x00, 0x6C, 0x6C, 0x00, 0x00,      // Code for char :
  0x00, 0xEC, 0x6C, 0x00, 0x00,      // Code for char ;
  0x08, 0x14, 0x22, 0x41, 0x00,      // Code for char <
  0x24, 0x24, 0x24, 0x24, 0x24,      // Code for char =
  0x00, 0x41, 0x22, 0x14, 0x08,      // Code for char >
  0x02, 0x01, 0x59, 0x09, 0x06,      // Code for char ?
  0x3E, 0x41, 0x5D, 0x55, 0x1E,      // Code for char @
  0x7E, 0x11, 0x11, 0x11, 0x7E,      // Code for char A
  0x7F, 0x49, 0x49, 0x49, 0x36,      // Code for char B
  0x3E, 0x41, 0x41, 0x41, 0x22,      // Code for char C
  0x7F, 0x41, 0x41, 0x41, 0x3E,      // Code for char D
  0x7F, 0x49, 0x49, 0x49, 0x41,      // Code for char E
  0x7F, 0x09, 0x09, 0x09, 0x01,      // Code for char F
  0x3E, 0x41, 0x49, 0x49, 0x7A,      // Code for char G
  0x7F, 0x08, 0x08, 0x08, 0x7F,      // Code for char H
  0x00, 0x41, 0x7F, 0x41, 0x00,      // Code for char I
  0x30, 0x40, 0x40, 0x40, 0x3F,      // Code for char J
  0x7F, 0x08, 0x14, 0x22, 0x41,      // Code for char K
  0x7F, 0x40, 0x40, 0x40, 0x40,      // Code for char L
  0x7F, 0x02, 0x04, 0x02, 0x7F,      // Code for char M
  0x7F, 0x02, 0x04, 0x08, 0x7F,      // Code for char N
  0x3E, 0x41, 0x41, 0x41, 0x3E,      // Code for char O
  0x7F, 0x09, 0x09, 0x09, 0x06,      // Code for char P
  0x3E, 0x41, 0x51, 0x21, 0x5E,      // Code for char Q
  0x7F, 0x09, 0x09, 0x19, 0x66,      // Code for char R
  0x26, 0x49, 0x49, 0x49, 0x32,      // Code for char S
  0x01, 0x01, 0x7F, 0x01, 0x01,      // Code for char T
  0x3F, 0x40, 0x40, 0x40, 0x3F,      // Code for char U
  0x1F, 0x20, 0x40, 0x20, 0x1F,      // Code for char V
  0x3F, 0x40, 0x3C, 0x40, 0x3F,      // Code for char W
  0x63, 0x14, 0x08, 0x14, 0x63,      // Code for char X
  0x07, 0x08, 0x70, 0x08, 0x07,      // Code for char Y
  0x71, 0x49, 0x45, 0x43, 0x00,      // Code for char Z
  0x00, 0x7F, 0x41, 0x41, 0x00,      // Code for char [
  0x02, 0x04, 0x08, 0x10, 0x20,      // Code for <BackSlash>
  0x00, 0x41, 0x41, 0x7F, 0x00,      // Code for char ]
  0x04, 0x02, 0x01, 0x02, 0x04,      // Code for char ^
  0x80, 0x80, 0x80, 0x80, 0x80,      // Code for char _
  0x00, 0x03, 0x07, 0x00, 0x00,      // Code for char `
  0x20, 0x54, 0x54, 0x54, 0x78,      // Code for char a
  0x7F, 0x44, 0x44, 0x44, 0x38,      // Code for char b
  0x38, 0x44, 0x44, 0x44, 0x28,      // Code for char c
  0x38, 0x44, 0x44, 0x44, 0x7F,      // Code for char d
  0x38, 0x54, 0x54, 0x54, 0x08,      // Code for char e
  0x08, 0x7E, 0x09, 0x09, 0x00,      // Code for char f
  0x18, 0xA4, 0xA4, 0xA4, 0x7C,      // Code for char g
  0x7F, 0x04, 0x04, 0x78, 0x00,      // Code for char h
  0x00, 0x00, 0x7D, 0x40, 0x00,      // Code for char i
  0x40, 0x80, 0x84, 0x7D, 0x00,      // Code for char j
  0x7F, 0x10, 0x28, 0x44, 0x00,      // Code for char k
  0x00, 0x00, 0x7F, 0x40, 0x00,      // Code for char l
  0x7C, 0x04, 0x18, 0x04, 0x78,      // Code for char m
  0x7C, 0x04, 0x04, 0x78, 0x00,      // Code for char n
  0x38, 0x44, 0x44, 0x44, 0x38,      // Code for char o
  0xFC, 0x44, 0x44, 0x44, 0x38,      // Code for char p
  0x38, 0x44, 0x44, 0x44, 0xFC,      // Code for char q
  0x44, 0x78, 0x44, 0x04, 0x08,      // Code for char r
  0x08, 0x54, 0x54, 0x54, 0x20,      // Code for char s
  0x04, 0x3E, 0x44, 0x24, 0x00,      // Code for char t
  0x3C, 0x40, 0x20, 0x7C, 0x00,      // Code for char u
  0x1C, 0x20, 0x40, 0x20, 0x1C,      // Code for char v
  0x3C, 0x60, 0x30, 0x60, 0x3C,      // Code for char w
  0x6C, 0x10, 0x10, 0x6C, 0x00,      // Code for char x
  0x9C, 0xA0, 0x60, 0x3C, 0x00,      // Code for char y
  0x64, 0x54, 0x54, 0x4C, 0x00,      // Code for char z
  0x08, 0x3E, 0x41, 0x41, 0x00,      // Code for char {
  0x00, 0x00, 0x77, 0x00, 0x00,      // Code for char |
  0x00, 0x41, 0x41, 0x3E, 0x08,      // Code for char }
  0x02, 0x01, 0x02, 0x01, 0x00,      // Code for char ~
  0x06, 0x09, 0x09, 0x06, 0x00       // Code for <Degrees>
};

static const uint8_t font2[] = {
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
192, 192, 192, 192, 192, 192, 192, 192, 192, 0, 192, 192, 192, 0, 0, 0,
216, 216, 144, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
108, 108, 108, 108, 254, 254, 108, 254, 254, 108, 108, 108, 108, 0, 0, 0,
24, 24, 124, 254, 210, 248, 124, 22, 150, 254, 124, 24, 24, 0, 0, 0,
0, 198, 198, 12, 12, 24, 24, 48, 48, 96, 96, 198, 198, 0, 0, 0,
56, 124, 108, 124, 56, 120, 108, 205, 197, 198, 206, 125, 125, 0, 0, 0,
0, 192, 192, 128, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
48, 48, 96, 96, 192, 192, 192, 192, 192, 192, 96, 96, 48, 48, 0, 0,
192, 192, 96, 96, 48, 48, 48, 48, 48, 48, 96, 96, 192, 192, 0, 0,
0, 0, 198, 198, 108, 56, 254, 254, 56, 108, 198, 198, 0, 0, 0, 0,
0, 0, 48 ,48, 48, 48, 252, 252, 48, 48, 48, 48, 0, 0, 0, 0,
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 224, 224, 96, 192, 128, 0,
0, 0, 0, 0, 0, 0, 248, 248, 0, 0, 0, 0, 0, 0, 0, 0,
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 224, 224, 224, 0, 0, 0, 0,
0, 0, 6, 6, 12, 12, 24, 24, 48, 48, 96, 96, 192, 192, 0, 0,
124, 254, 198, 198, 206, 206, 222, 246, 230, 230, 198, 254, 124, 0, 0, 0,
48, 112, 112, 48, 48, 48, 48, 48, 48, 48, 48, 252, 252, 0, 0, 0,
124, 254, 198, 6, 6, 62, 124, 224, 192, 192, 192, 254, 254, 0, 0, 0,
124, 254, 198, 6, 6, 30, 28, 6, 6, 6, 198, 254, 124, 0, 0, 0,
12, 28, 28, 60, 108, 108, 204, 204, 254, 254, 12, 12, 12, 0, 0, 0,
252, 252, 192, 192, 192, 248, 252, 14, 6, 6, 198, 254, 124, 0, 0, 0,
124, 254, 198, 192, 192, 248, 252, 206, 198, 198, 230, 254, 124, 0, 0, 0,
254, 254, 198, 12, 12, 24, 24, 48, 48, 48, 48, 48, 48, 0, 0, 0,
124, 254, 198, 198, 198, 254, 124, 198, 198, 198, 198, 254, 124, 0, 0, 0,
124, 254, 198, 198, 198, 254, 126, 6, 6, 6, 6, 254, 124, 0, 0, 0,
0, 0, 0, 192, 192, 0, 0, 0, 192, 192, 0, 0, 0, 0, 0, 0,
0, 0, 0, 96, 96, 0, 0, 0, 96, 96, 192, 128, 0, 0, 0, 0,
24, 24, 48, 48, 96, 96, 192, 192, 96, 96, 48, 48, 24, 24, 0, 0,
0, 0, 0, 0, 248, 248, 0, 0, 248, 248, 0, 0, 0, 0, 0, 0,
192, 192, 96, 96, 48, 48, 24, 24, 48, 48, 96, 96, 192, 192, 0, 0,
124, 254, 198, 6, 14, 28, 56, 48, 48, 0, 0, 48, 48, 0, 0, 0,
0, 0, 0, 0, 28, 34, 78, 82, 82, 82, 78, 32, 31, 0, 0, 0, 
16, 56, 108, 108, 198, 198, 254, 254, 198, 198, 198, 198, 198, 0, 0, 0,
252, 254, 198, 198, 198, 254, 252, 198, 198, 198, 198, 254, 252, 0, 0, 0,
124, 254, 198, 192, 192, 192, 192, 192, 192, 192, 198, 254, 124, 0, 0, 0,
248, 252, 206, 198, 198, 198, 198, 198, 198, 198, 206, 252, 248, 0, 0, 0,
254, 254, 192, 192, 192, 248, 248, 192, 192, 192, 192, 254, 254, 0, 0, 0,
254, 254, 192, 192, 192, 248, 248, 192, 192, 192, 192, 192, 192, 0, 0, 0,
56, 124, 230, 192, 192, 192, 222, 222, 198, 198, 238, 124, 56, 0, 0, 0,
198, 198, 198, 198, 198, 254, 254, 198, 198, 198, 198, 198, 198, 0, 0, 0,
48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 0, 0, 0,
6, 6, 6, 6, 6, 6, 6, 6, 6, 198, 238, 124, 56, 0, 0, 0,
198, 198, 204, 204, 216, 248, 240, 216, 216, 204, 204, 198, 198, 0, 0, 0,
192, 192, 192, 192, 192, 192, 192, 192, 192, 192, 192, 254, 254, 0, 0, 0,
195, 231, 231, 255, 255, 255, 219, 219, 219, 219, 219, 195, 195, 0, 0, 0,
198, 198, 230, 230, 230, 246, 246, 222, 222, 206, 206, 198, 198, 0, 0, 0,
56, 124, 238, 198, 198, 198, 198, 198, 198, 198, 238, 124, 56, 0, 0, 0,
248, 252, 206, 198, 206, 252, 248, 192, 192, 192, 192, 192, 192, 0, 0, 0,
56, 124, 238, 198, 198, 198, 198, 198, 214, 222, 204, 126, 58, 0, 0, 0,
248, 252, 206, 198, 206, 252, 248, 216, 216, 204, 204, 198, 198, 0, 0, 0,
60, 126, 230, 192, 224, 120, 60, 14, 6, 6, 206, 252, 120, 0, 0, 0,
252, 252, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 0, 0, 0,
198, 198, 198, 198, 198, 198, 198, 198, 198, 198, 198, 254, 124, 0, 0, 0,
198, 198, 198, 198, 198, 198, 198, 108, 108, 56, 56, 16, 16, 0, 0, 0,
195, 219, 219, 219, 219, 219, 255, 255, 255, 231, 231, 195, 195, 0, 0, 0,
198, 198, 108, 108, 56, 56, 16, 56, 56, 108, 108, 198, 198, 0, 0, 0,
204, 204, 204, 204, 120, 120, 48, 48, 48, 48, 48, 48, 48, 0, 0, 0,
254, 254, 12, 24, 24, 48, 48, 96, 96, 192, 192, 254, 254, 0, 0, 0,
240, 240, 192, 192, 192, 192, 192, 192, 192, 192, 192, 240, 240, 0, 0, 0,
128, 192, 192, 96, 32, 48, 16, 24, 8, 12, 4, 6, 2, 0, 0, 0,
240, 240, 48, 48, 48, 48, 48, 48, 48, 48, 48, 240, 240, 0, 0, 0,
16, 16, 56, 56, 108, 108, 198, 198, 0, 0, 0, 0, 0, 0, 0, 0,
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 254, 0, 0, 0,
0, 192, 192, 64, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
0, 0, 0, 0, 120, 252, 204, 124, 252, 204, 204, 254, 119, 0, 0, 0,
192, 192, 192, 192, 248, 252, 204, 204, 204, 204, 204, 252, 248, 0, 0, 0,
0, 0, 0, 0, 120, 252, 196, 192, 192, 192, 196, 252, 120, 0, 0, 0,
12, 12, 12, 12, 124, 252, 204, 204, 204, 204, 204, 252, 124, 0, 0, 0,
0, 0, 0, 0, 120, 252, 204, 252, 248, 192, 196, 252, 120, 0, 0, 0,
12, 60, 48, 48, 252, 252, 48, 48, 48, 48, 48, 48, 48, 0, 0, 0,
0, 0, 0, 0, 120, 252, 204, 204, 204, 252, 124, 12, 140, 252, 120, 0,
192, 192, 192, 192, 248, 252, 204, 204, 204, 204, 204, 204, 204, 0, 0, 0,
48, 48, 0, 0, 112, 112, 48, 48, 48, 48, 48, 120, 120, 0, 0, 0,
24, 0, 0, 0, 24, 24, 24, 24, 24, 24, 24, 24, 152, 248, 112, 0,
192, 192, 196, 204, 216, 240, 224, 240, 216, 200, 204, 198, 198, 0, 0, 0,
224, 224, 96, 96, 96, 96, 96, 96, 96, 96, 96, 240, 240, 0, 0, 0,
0, 0, 0, 0, 236, 254, 214, 214, 214, 198, 198, 198, 198, 0, 0, 0,
0, 0, 0, 0, 248, 252, 204, 204, 204, 204, 204, 204, 204, 0, 0, 0,
0, 0, 0, 0, 120, 252, 204, 204, 204, 204, 204, 252, 120, 0, 0, 0,
0, 0, 0, 0, 248, 252, 204, 204, 204, 204, 252, 248, 192, 192, 192, 0,
0, 0, 0, 0, 124, 252, 204, 204, 204, 204, 252, 124, 12, 12, 6, 0,
0, 0, 0, 0, 216, 252, 238, 198, 192, 192, 192, 192, 192, 0, 0, 0,
0, 0, 0, 0, 120, 252, 196, 240, 120, 12, 140, 252, 120, 0, 0, 0,
48, 48, 48, 48, 252, 252, 48, 48, 48, 48, 48, 28, 12, 0, 0, 0,
0, 0, 0, 0, 204, 204, 204, 204, 204, 204, 204, 252, 120, 0, 0, 0,
0, 0, 0, 0, 198, 198, 198, 108, 108, 56, 56, 16, 16, 0, 0, 0,
0, 0, 0, 0, 198, 198, 198, 214, 214, 124, 108, 108, 40, 0, 0, 0,
0, 0, 0, 0, 198, 198, 108, 124, 56, 124, 108, 198, 198, 0, 0, 0,
0, 0, 0, 0, 204, 204, 204, 204, 204, 252, 124, 12, 140, 252, 120, 0,
0, 0, 0, 0, 252, 252, 24, 24, 48, 48, 96, 252, 252, 0, 0, 0,
0, 12, 24, 24, 24, 24, 24, 48, 24, 24, 24, 24, 24, 12, 0, 0,
24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 0,
0, 48, 24, 24, 24, 24, 24, 12, 24, 24, 24, 24, 24, 48, 0, 0,
0, 0, 0, 0, 0, 96, 226,158,12,0, 0, 0, 0, 0, 0, 0,
120, 252, 204, 204, 204, 204, 252, 120, 0, 0, 0, 0, 0, 0, 0, 0
};

int drawChar1(TFT_t * dev, uint8_t sizew, uint8_t sizeht, int16_t x, int16_t y, unsigned char c, uint16_t color) {
	c -= 32;
static const uint8_t fstyle = 0;
static const uint8_t width = 240;
static const uint8_t height = 240;
int fsw = 5; //iod09 font selection
int fsh = 8; //iod09 font selection
static uint16_t bg = BLACK;
  int crad;
  //int co;
  //if (fstyle == 1 || fstyle == 2 || fstyle == 3) crad = sizew >> 1;
  //if (fstyle == DOTMATRIXLED) co = crad * 68 / 100;
  if (sizew > 3)crad --;
  if (dev->_font_direction != 3 && dev->_font_direction != 2)
  {
    if ((x >= width) || (y >= height) || ((x + (fsw + 1) * sizew - 1) < 0) || ((y + fsh * sizeht - 1) < 0))
    {
      return x+(sizew*6);
    }
  }
  for (int8_t i = 0; i < 6; i++ )
  {
    uint8_t tcol;
    if (i == (fsw))
    {
      tcol = 0x0;
    }
    else
    {
      tcol = font1[(c * 5) + i];
    }
    for (int8_t j = 0; j < 8; j++)
    {
      if (i == 5) tcol = 0;
      if (tcol & 0x1)
      {
        if (sizew == 1 && sizeht == 1)
        {
          if (y + j > 319 && (dev->_font_direction == 3 || dev->_font_direction == 2))
          {
            lcdDrawPixel(dev, x + i, y + j - 320, color);
          }
          else
          {
            lcdDrawPixel(dev, x + i, y + j, color);
          }
        }
        else
        {
          if (fstyle == 0) lcdDrawFillRect(dev, x + (i * sizew), y + (j * sizeht), (sizew + x) + (i * sizew) - 1, (sizeht + y) + (j * sizeht) - 1, color);
          /*
		  if (fstyle == 2)Circle(x + (i * sizew) + crad, y + (j * sizeht) + crad, crad, color);
          if (fstyle == 1)CircleFilled(x + (i * sizew) + crad, y + (j * sizeht) + crad, crad, color);
          if (fstyle == 3)
          {
            CircleFilled(x + (i * sizew) + crad, y + (j * sizeht) + crad, crad, color);
            CircleFilled(x + (i * sizew) + co, y + (j * sizeht) + co, crad / 3, WHITE);
          }
          if (fstyle == 4)RectangleFilled(x + (i * sizew), y + (j * sizeht), (sizew + x) + (i * sizew) - 2, (sizeht + y) + (j * sizeht) - 2, color);
          if (fstyle == 5)
          {
            uint16_t fadcol = color;
            fadcol = HighlightColors(fadcol, 10) & 0xffff;
            int step = 60 / (sizew / 2);
            if (step < 1) step = 1;
            for (int n = sizew / 2; n > -1; n --)
            {
              Rectangle(x + (i * sizew) + n, y + (j * sizeht) + n, (sizew + x) + (i * sizew) - 1 - n, (sizeht + y) + (j * sizeht) - 1 - n, fadcol);
              fadcol = HighlightColors(fadcol, step) >> 16;
            }
          }
		  */
        }
      }
      else if (bg != color)
      {
        if (sizew == 1 && sizeht == 1)
        {
          if (y + j > 319 && (dev->_font_direction == 3 || dev->_font_direction == 2))
          {
            lcdDrawPixel(dev, x + i, y + j - 320, bg);
          }
          else
          {
            lcdDrawPixel(dev, x + i, y + j, bg);
          }
        }
        else
        {
          lcdDrawFillRect(dev, x + i * sizew, y + j * sizeht, (sizew + x) + i * sizew - 1, (sizeht + y) + j * sizeht - 1, bg);
        }
      }
      tcol >>= 1;
    }
  }
  return x+(sizew*6);
}

int drawChar2(TFT_t * dev, uint8_t sizeht , uint8_t sizew, int16_t x, int16_t y, unsigned char c, uint16_t color) {
	c -= 32;
uint8_t rotation = dev->_font_direction;
int fsw = 8; //iod09 font selection
int fwh = 16; //iod09 font selection

	int next = 0;
	if (dev->_font_direction == 0) {
		next = x + fsw * sizew + sizew;
	} else if (dev->_font_direction == 2) {
		next = x - fsw * sizew + sizew;
	} else if (dev->_font_direction == 1) {
		next = y + fwh * sizeht + sizeht;
	} else if (dev->_font_direction == 3) {
		next = y - fwh * sizeht + sizeht;
	}

	//if(rotation != 3 && rotation != 2){
	//if((x >= width)||(y >= height)||((x + (fsw + 1) * sizew - 1) < 0)||((y + fsh * sizeht - 1) < 0)){  
	//return;
	//}
	//}
	for (int8_t j = 0; j < 16; j++ ) {
		uint8_t trow;
		trow = font2[(c * 16) + j];
		for (int8_t i = 0; i < (fsw + 1); i++)
		{
			if (i == (fsw))
			{
				trow = 0x00;
			}
			if (trow & 0x80)
			{
				if (sizew == 1 && sizeht == 1)
				{
					if(y + j > 159 && (rotation == 3 || rotation == 2))
					{
						lcdDrawPixel(dev, x + i, y + j - 160, color);
					}
					else
					{
						lcdDrawPixel(dev, x + i, y + j, color);
					}
				}
				else
				{
					lcdDrawFillRect(dev, x + (i * sizew), y + (j * sizeht), (sizew + x) + (i * sizew) - 1, (sizeht + y) + (j * sizeht) - 1, color);
				}
			}
			else if (dev->_font_fill_color != color)
			{
				if (sizew == 1 && sizeht == 1)
				{
					if(y + j > 159 && (rotation == 3 || rotation == 2))
					{
						lcdDrawPixel(dev, x + i, y + j - 160, dev->_font_fill_color);
					}
					else
					{
						lcdDrawPixel(dev, x + i, y + j, dev->_font_fill_color);
					}
				}
				else
				{
					lcdDrawFillRect(dev, x + (i* sizew), y + (j * sizeht), (sizew + x) + (i* sizew) - 1, (sizeht + y) + (j * sizeht) - 1, dev->_font_fill_color);
				}
			}
			trow <<= 1;
		}
	}
	if (next < 0) next = 0;
	return next;
}

// Draw ASCII character
// x:X coordinate
// y:Y coordinate
// ascii: ascii code
// color:color
int lcdDrawChar(TFT_t * dev, FontxFile *fxs, uint16_t x, uint16_t y, uint8_t ascii, uint16_t color) {
	uint16_t xx,yy,bit,ofs;
	unsigned char fonts[128]; // font pattern
	unsigned char pw, ph;
	int h,w;
	uint16_t mask;
	bool rc;

	if(_DEBUG_)printf("_font_direction=%d\n",dev->_font_direction);
	rc = GetFontx(fxs, ascii, fonts, &pw, &ph);
	if(_DEBUG_)printf("GetFontx rc=%d pw=%d ph=%d\n",rc,pw,ph);
	if (!rc) return 0;

	int16_t xd1 = 0;
	int16_t yd1 = 0;
	int16_t xd2 = 0;
	int16_t yd2 = 0;
	uint16_t xss = 0;
	uint16_t yss = 0;
	int16_t xsd = 0;
	int16_t ysd = 0;
	int16_t next = 0;
	uint16_t x0  = 0;
	uint16_t x1  = 0;
	uint16_t y0  = 0;
	uint16_t y1  = 0;
	if (dev->_font_direction == 0) {
		xd1 = +1;
		yd1 = +1; //-1;
		xd2 =  0;
		yd2 =  0;
		xss =  x;
		yss =  y - (ph - 1);
		xsd =  1;
		ysd =  0;
		next = x + pw;

		x0	= x;
		y0	= y - (ph-1);
		x1	= x + (pw-1);
		y1	= y;
	} else if (dev->_font_direction == 2) {
		xd1 = -1;
		yd1 = -1; //+1;
		xd2 =  0;
		yd2 =  0;
		xss =  x;
		yss =  y + ph + 1;
		xsd =  1;
		ysd =  0;
		next = x - pw;

		x0	= x - (pw-1);
		y0	= y;
		x1	= x;
		y1	= y + (ph-1);
	} else if (dev->_font_direction == 1) {
		xd1 =  0;
		yd1 =  0;
		xd2 = -1;
		yd2 = +1; //-1;
		xss =  x + ph;
		yss =  y;
		xsd =  0;
		ysd =  1;
		next = y + pw; //y - pw;

		x0	= x;
		y0	= y;
		x1	= x + (ph-1);
		y1	= y + (pw-1);
	} else if (dev->_font_direction == 3) {
		xd1 =  0;
		yd1 =  0;
		xd2 = +1;
		yd2 = -1; //+1;
		xss =  x - (ph - 1);
		yss =  y;
		xsd =  0;
		ysd =  1;
		next = y - pw; //y + pw;

		x0	= x - (ph-1);
		y0	= y - (pw-1);
		x1	= x;
		y1	= y;
	}

	if (dev->_font_fill) lcdDrawFillRect(dev, x0, y0, x1, y1, dev->_font_fill_color);

	int bits;
	if(_DEBUG_)printf("xss=%d yss=%d\n",xss,yss);
	ofs = 0;
	yy = yss;
	xx = xss;
	for(h=0;h<ph;h++) {
		if(xsd) xx = xss;
		if(ysd) yy = yss;
		//for(w=0;w<(pw/8);w++) {
		bits = pw;
		for(w=0;w<((pw+4)/8);w++) {
			mask = 0x80;
			for(bit=0;bit<8;bit++) {
				bits--;
				if (bits < 0) continue;
				//if(_DEBUG_)printf("xx=%d yy=%d mask=%02x fonts[%d]=%02x\n",xx,yy,mask,ofs,fonts[ofs]);
				if (fonts[ofs] & mask) {
					lcdDrawPixel(dev, xx, yy, color);
				} else {
					//if (dev->_font_fill) lcdDrawPixel(dev, xx, yy, dev->_font_fill_color);
				}
				if (h == (ph-2) && dev->_font_underline)
					lcdDrawPixel(dev, xx, yy, dev->_font_underline_color);
				if (h == (ph-1) && dev->_font_underline)
					lcdDrawPixel(dev, xx, yy, dev->_font_underline_color);
				xx = xx + xd1;
				yy = yy + yd2;
				mask = mask >> 1;
			}
			ofs++;
		}
		yy = yy + yd1;
		xx = xx + xd2;
	}

	if (next < 0) next = 0;
	return next;
}

int lcdDrawString(TFT_t * dev, FontxFile *fx, uint16_t x, uint16_t y, uint8_t * ascii, uint16_t color) {
	int length = strlen((char *)ascii);
	if(_DEBUG_)printf("lcdDrawString length=%d\n",length);
	for(int i=0;i<length;i++) {
		if(_DEBUG_)printf("ascii[%d]=%x x=%d y=%d\n",i,ascii[i],x,y);
		if (dev->_font_direction == 0)
			x = lcdDrawChar(dev, fx, x, y, ascii[i], color);
		if (dev->_font_direction == 1)
			y = lcdDrawChar(dev, fx, x, y, ascii[i], color);
		if (dev->_font_direction == 2)
			x = lcdDrawChar(dev, fx, x, y, ascii[i], color);
		if (dev->_font_direction == 3)
			y = lcdDrawChar(dev, fx, x, y, ascii[i], color);
	}
	if (dev->_font_direction == 0) return x;
	if (dev->_font_direction == 2) return x;
	if (dev->_font_direction == 1) return y;
	if (dev->_font_direction == 3) return y;
	return 0;
}

/* iod 09 large font */
int lcdDrawString2(TFT_t * dev, uint8_t sizeht , uint8_t sizew, uint16_t x, uint16_t y, uint8_t * ascii, uint16_t color) {
	int length = strlen((char *)ascii);
	//printf("lcdDrawString length=%d\n",length);
	for(int i=0;i<length;i++) {
		//printf("ascii[%d]=%x x=%d y=%d\n",i,ascii[i],x,y);
		if (dev->_font_direction == 0)
			x = drawChar2(dev, sizeht, sizew, x, y, ascii[i], color);
		if (dev->_font_direction == 1)
			y = drawChar2(dev, sizeht, sizew, x, y, ascii[i], color);
		if (dev->_font_direction == 2)
			x = drawChar2(dev, sizeht, sizew, x, y, ascii[i], color);
		if (dev->_font_direction == 3)
			y = drawChar2(dev, sizeht, sizew, x, y, ascii[i], color);
	}
	if (dev->_font_direction == 0) return x;
	if (dev->_font_direction == 2) return x;
	if (dev->_font_direction == 1) return y;
	if (dev->_font_direction == 3) return y;
	return 0;
}

/* iod 09 small font */
int lcdDrawString3(TFT_t * dev, uint8_t sizeht , uint8_t sizew, uint16_t x, uint16_t y, uint8_t * ascii, uint16_t color) {
	int length = strlen((char *)ascii);
	//printf("lcdDrawString length=%d\n",length);
	for(int i=0;i<length;i++) {
		//printf("ascii[%d]=%x x=%d y=%d\n",i,ascii[i],x,y);
		if (dev->_font_direction == 0)
			x = drawChar1(dev, sizeht, sizew, x, y, ascii[i], color);
		if (dev->_font_direction == 1)
			y = drawChar1(dev, sizeht, sizew, x, y, ascii[i], color);
		if (dev->_font_direction == 2)
			x = drawChar1(dev, sizeht, sizew, x, y, ascii[i], color);
		if (dev->_font_direction == 3)
			y = drawChar1(dev, sizeht, sizew, x, y, ascii[i], color);
	}
	if (dev->_font_direction == 0) return x;
	if (dev->_font_direction == 2) return x;
	if (dev->_font_direction == 1) return y;
	if (dev->_font_direction == 3) return y;
	return 0;
}

// Draw Non-Alphanumeric character
// x:X coordinate
// y:Y coordinate
// code: charcter code
// color:color
int lcdDrawCode(TFT_t * dev, FontxFile *fx, uint16_t x,uint16_t y,uint8_t code,uint16_t color) {
	if(_DEBUG_)printf("code=%x x=%d y=%d\n",code,x,y);
	if (dev->_font_direction == 0)
		x = lcdDrawChar(dev, fx, x, y, code, color);
	if (dev->_font_direction == 1)
		y = lcdDrawChar(dev, fx, x, y, code, color);
	if (dev->_font_direction == 2)
		x = lcdDrawChar(dev, fx, x, y, code, color);
	if (dev->_font_direction == 3)
		y = lcdDrawChar(dev, fx, x, y, code, color);
	if (dev->_font_direction == 0) return x;
	if (dev->_font_direction == 2) return x;
	if (dev->_font_direction == 1) return y;
	if (dev->_font_direction == 3) return y;
	return 0;
}

#if 0
// Draw UTF8 character
// x:X coordinate
// y:Y coordinate
// utf8: UTF8 code
// color:color
int lcdDrawUTF8Char(TFT_t * dev, FontxFile *fx, uint16_t x,uint16_t y,uint8_t *utf8,uint16_t color) {
	uint16_t sjis[1];

	sjis[0] = UTF2SJIS(utf8);
	if(_DEBUG_)printf("sjis=%04x\n",sjis[0]);
	return lcdDrawSJISChar(dev, fx, x, y, sjis[0], color);
}

// Draw UTF8 string
// x:X coordinate
// y:Y coordinate
// utfs: UTF8 string
// color:color
int lcdDrawUTF8String(TFT_t * dev, FontxFile *fx, uint16_t x, uint16_t y, unsigned char *utfs, uint16_t color) {

	int i;
	int spos;
	uint16_t sjis[64];
	spos = String2SJIS(utfs, strlen((char *)utfs), sjis, 64);
	if(_DEBUG_)printf("spos=%d\n",spos);
	for(i=0;i<spos;i++) {
		if(_DEBUG_)printf("sjis[%d]=%x y=%d\n",i,sjis[i],y);
		if (dev->_font_direction == 0)
			x = lcdDrawSJISChar(dev, fx, x, y, sjis[i], color);
		if (dev->_font_direction == 1)
			y = lcdDrawSJISChar(dev, fx, x, y, sjis[i], color);
		if (dev->_font_direction == 2)
			x = lcdDrawSJISChar(dev, fx, x, y, sjis[i], color);
		if (dev->_font_direction == 3)
			y = lcdDrawSJISChar(dev, fx, x, y, sjis[i], color);
	}
	if (dev->_font_direction == 0) return x;
	if (dev->_font_direction == 2) return x;
	if (dev->_font_direction == 1) return y;
	if (dev->_font_direction == 3) return y;
	return 0;
}
#endif

// Set font direction
// dir:Direction
void lcdSetFontDirection(TFT_t * dev, uint16_t dir) {
	dev->_font_direction = dir;
}

// Set font filling
// color:fill color
void lcdSetFontFill(TFT_t * dev, uint16_t color) {
	dev->_font_fill = true;
	dev->_font_fill_color = color;
}

// UnSet font filling
void lcdUnsetFontFill(TFT_t * dev) {
	dev->_font_fill = false;
}

// Set font underline
// color:frame color
void lcdSetFontUnderLine(TFT_t * dev, uint16_t color) {
	dev->_font_underline = true;
	dev->_font_underline_color = color;
}

// UnSet font underline
void lcdUnsetFontUnderLine(TFT_t * dev) {
	dev->_font_underline = false;
}

// Backlight OFF
void lcdBacklightOff(TFT_t * dev) {
	if(dev->_bl >= 0) {
		gpio_set_level( dev->_bl, 0 );
	}
}

// Backlight ON
void lcdBacklightOn(TFT_t * dev) {
	if(dev->_bl >= 0) {
		gpio_set_level( dev->_bl, 1 );
	}
}

// Display Inversion Off
void lcdInversionOff(TFT_t * dev) {
	spi_master_write_command(dev, 0x20);	//Display Inversion Off
}

// Display Inversion On
void lcdInversionOn(TFT_t * dev) {
	spi_master_write_command(dev, 0x21);	//Display Inversion On
}

// #########################################################################
// Draw an arc with a defined thickness
// #########################################################################

// x,y == coords of centre of arc
// start_angle = 0 - 359
// seg_count = number of 3 degree segments to draw (120 => 360 degree arc)
// rx = x axis radius
// yx = y axis radius
// w  = width (thickness) of arc in pixels
// color = 16 bit color value
// Note if rx and ry are the same an arc of a circle is drawn
void lcdFillArc(TFT_t * dev, int x, int y, int start_angle, int seg_count, int rx, int ry, int w, uint16_t color)
{
	uint8_t seg = 3; // Segments are 3 degrees wide = 120 segments for 360 degrees
	uint8_t inc = 6; // Draw segments every 3 degrees, increase to 6 for segmented ring

	// Draw color blocks every inc degrees
	for (int i = start_angle; i < start_angle + seg * seg_count; i += inc)
	{
		// Calculate pair of coordinates for segment start
		float sx = cos((i - 90) * DEG2RAD);
		float sy = sin((i - 90) * DEG2RAD);
		uint16_t x0 = sx * (rx - w) + x;
		uint16_t y0 = sy * (ry - w) + y;
		uint16_t x1 = sx * rx + x;
		uint16_t y1 = sy * ry + y;

		// Calculate pair of coordinates for segment end
		float sx2 = cos((i + seg - 90) * DEG2RAD);
		float sy2 = sin((i + seg - 90) * DEG2RAD);
		int x2 = sx2 * (rx - w) + x;
		int y2 = sy2 * (ry - w) + y;
		int x3 = sx2 * rx + x;
		int y3 = sy2 * ry + y;

		lcdFillTriangle(dev, x0, y0, x1, y1, x2, y2, color);
		lcdFillTriangle(dev, x1, y1, x2, y2, x3, y3, color);
	}
}

//NOTE: Copy of lcdFillArc with parameterized segment size
void lcdFillArc3(TFT_t * dev, int x, int y, int start_angle, int seg_count, int rx, int ry, int w, uint16_t color, uint8_t segments)
{
	uint8_t seg = segments; // Segments
	uint8_t inc = segments * 2; // Draw segmented ring

	// Draw color blocks every inc degrees
	for (int i = start_angle; i < start_angle + seg * seg_count; i += inc)
	{
		// Calculate pair of coordinates for segment start
		float sx = cos((i - 90) * DEG2RAD);
		float sy = sin((i - 90) * DEG2RAD);
		uint16_t x0 = sx * (rx - w) + x;
		uint16_t y0 = sy * (ry - w) + y;
		uint16_t x1 = sx * rx + x;
		uint16_t y1 = sy * ry + y;

		// Calculate pair of coordinates for segment end
		float sx2 = cos((i + seg - 90) * DEG2RAD);
		float sy2 = sin((i + seg - 90) * DEG2RAD);
		int x2 = sx2 * (rx - w) + x;
		int y2 = sy2 * (ry - w) + y;
		int x3 = sx2 * rx + x;
		int y3 = sy2 * ry + y;

		lcdFillTriangle(dev, x0, y0, x1, y1, x2, y2, color);
		lcdFillTriangle(dev, x1, y1, x2, y2, x3, y3, color);
	}
}

// #########################################################################
// Draw a circular or elliptical arc with a defined thickness
// #########################################################################

// x,y == coords of centre of arc
// start_angle = 0 - 359
// seg_count = number of 3 degree segments to draw (120 => 360 degree arc)
// rx = x axis radius
// yx = y axis radius
// w  = width (thickness) of arc in pixels
// color = 16 bit color value
// Note if rx and ry are the same then an arc of a circle is drawn
void lcdFillArc2(TFT_t * dev, int x, int y, int start_angle, int seg_count, int rx, int ry, int w, uint16_t color)
{
	uint8_t seg = 3; // Segments are 3 degrees wide = 120 segments for 360 degrees
	uint8_t inc = 6; // Draw segments every 3 degrees, increase to 6 for segmented ring

	// Calculate first pair of coordinates for segment start
	float sx = cos((start_angle - 90) * DEG2RAD);
	float sy = sin((start_angle - 90) * DEG2RAD);
	uint16_t x0 = sx * (rx - w) + x;
	uint16_t y0 = sy * (ry - w) + y;
	uint16_t x1 = sx * rx + x;
	uint16_t y1 = sy * ry + y;

	// Draw color blocks every inc degrees
	for (int i = start_angle; i < start_angle + seg * seg_count; i += inc)
	{
		// Calculate pair of coordinates for segment end
		float sx2 = cos((i + seg - 90) * DEG2RAD);
		float sy2 = sin((i + seg - 90) * DEG2RAD);
		int x2 = sx2 * (rx - w) + x;
		int y2 = sy2 * (ry - w) + y;
		int x3 = sx2 * rx + x;
		int y3 = sy2 * ry + y;

		lcdFillTriangle(dev, x0, y0, x1, y1, x2, y2, color);
		lcdFillTriangle(dev, x1, y1, x2, y2, x3, y3, color);

		// Copy segment end to sgement start for next segment
		x0 = x2;
		y0 = y2;
		x1 = x3;
		y1 = y3;
	}
}

void line_test(TFT_t * dev, uint16_t x, uint16_t y, uint16_t size, uint16_t * colors) {
	if (x+size > dev->_width) return;
	if (y >= dev->_height) return;

	uint16_t _x1 = x + dev->_offsetx;
	uint16_t _x2 = _x1 + size;
	uint16_t _y1 = y + dev->_offsety;
	uint16_t _y2 = _y1;

	spi_master_write_command(dev, 0x2A);	// set column(x) address
	spi_master_write_addr(dev, _x1, _x2);
	spi_master_write_command(dev, 0x2B);	// set Page(y) address
	spi_master_write_addr(dev, _y1, _y2);
	spi_master_write_command(dev, 0x2C);	//	Memory Write
	spi_master_write_colors(dev, colors, size);
}

void lcdUpdate(TFT_t * dev) {
	for (int i=0; i<240; ++i) {
		line_test(dev, 0, i, 240, &_frame_buffer[i*240]);
	}
}
