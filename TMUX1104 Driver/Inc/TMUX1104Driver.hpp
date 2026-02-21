#ifndef TMUX1104_HPP
#define TMUX1104_HPP

#include "stm32h7xx.h"
#include "CubeDefines.hpp"

enum class Camera : uint8_t{
	NONE = 0,
	CAMERA1,
	CAMERA2,
	CAMERA3,
	CAMERA4
};

class TMUX1104Driver{
	public:
		TMUX1104Driver(GPIO_TypeDef* a0_gpio, uint16_t a0_pin,
						GPIO_TypeDef* a1_gpio, uint16_t a1_pin,
						GPIO_TypeDef *enable_gpio, uint16_t enable_pin);
		void Enable();
		void Disable();
		void Select(Camera cam);

	private:

		//a0 LSB for decoder
		GPIO_TypeDef *a0_gpio_;
		uint16_t a0_pin_;
		//a1 MSB for decoder
		GPIO_TypeDef *a1_gpio_;
		uint16_t a1_pin_;
		//Enable for decoder
		GPIO_TypeDef *enable_gpio_;
		uint16_t enable_pin_;

		Camera current_cam;
		bool enabled;

};

#endif
