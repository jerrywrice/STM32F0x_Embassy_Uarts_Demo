# STM32F0x_Embassy_Uarts_Demo
Sample embassy (async) embedded app for STM32F0xx (Nucleo-STM32F091RC board)

In this version after initially cloning/fetching/checking-out and before building, edit the project's cargo.toml file per

# Revise following 'path = "../../embassy/embassy-stm32" to reference your local copy of the embassy source crate
embassy-stm32 = { version = "0.1.0", path = "../../embassy/embassy-stm32", features = [ "defmt", "memory-x", "stm32f091rc", "time-driver-tim2", "unstable-pac", "exti"] }

prior to building.

