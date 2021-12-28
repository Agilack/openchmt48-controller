/**
 * @file  hardware.c
 * @brief Registers read and write functions
 *
 * @authors Axel Paolillo Axel <axel.paolillo@agilack.fr>
 *          Saint-Genest Gwenael <gwen@agilack.fr>
 * @copyright Agilack (c) 2021
 *
 * @page License
 * This firmware is free software: you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License version 3 as
 * published by the Free Software Foundation. You should have received a
 * copy of the GNU Lesser General Public License along with this program,
 * see LICENSE.md file for more details.
 * This program is distributed WITHOUT ANY WARRANTY.
 */

#include "hardware.h"

/**
 * @brief Read the value of a 32bits memory mapped register
 *
 * @param reg Address of the register to read
 * @return u32 Value of the register
 */
u32 reg_rd(u32 reg)
{
    return ( *(volatile u32 *)reg );
}

/**
 * @brief Write a value to a 32bits memory mapped register
 *
 * @param reg Address of the register to write
 * @param value Value to write into the register
 */
void reg_wr(u32 reg, u32 value)
{
    *(volatile u32 *)reg = value;
}

/**
 * @brief Modify a 32bits memory mapped register by setting some bits
 *
 * @param reg Address of the register to write
 * @param value Mask of the bits to set
 */
void reg_set(u32 reg, u32 value)
{
    *(volatile u32 *)reg = ( *(volatile u32 *)reg | value );
}

/**
 * @brief Modify a 32bits memory mapped register by clearing some bits
 *
 * @param reg Address of the register to write
 * @param value Mask of the bits to clear
 */
void reg_clr(u32 reg, u32 value)
{
    *(volatile u32 *)reg = ( *(volatile u32 *)reg & ~value );
}

/**
 * @brief Read the value of a 16bits memory mapped register
 *
 * @param  reg Address of the register to read
 * @return u16 Value of the register
 */
u16 reg16_rd(u32 reg)
{
        return ( *(volatile u16 *)reg );
}


/**
 * @brief Write a value to a 16bits memory mapped register
 *
 * @param reg Address of the register to write
 * @param value Value to write into the register
 */
void reg16_wr(u32 reg, u16 value)
{
        *(volatile u16 *)reg = value;
}

/**
 * @brief Modify a 16bits memory mapped register by clearing some bits
 *
 * @param reg Address of the register to write
 * @param value Mask of the bits to clear
 */
void reg16_clr(u32 reg, u16 value)
{
        *(volatile u16 *)reg = ( *(volatile u16 *)reg & ~value );
}

/**
 * @brief Modify a 16bits memory mapped register by setting some bits
 *
 * @param reg Address of the register to write
 * @param value Value to write into the register
 */
void reg16_set(u32 reg, u16 value)
{
        *(volatile u16 *)reg = ( *(volatile u16 *)reg | value );
}