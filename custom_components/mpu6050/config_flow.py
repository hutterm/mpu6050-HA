import logging
from typing import Any

import serial
import voluptuous as vol

from homeassistant import config_entries
from homeassistant.config_entries import (
    ConfigEntry,
    ConfigFlow,
    ConfigFlowResult,
)
from homeassistant.core import HomeAssistant
from homeassistant.data_entry_flow import FlowResult
from homeassistant.exceptions import HomeAssistantError
from homeassistant.helpers import selector
from homeassistant.core import callback

import os
from smbus2 import SMBus

from .const import CONF_BUS_ADDRESS, DOMAIN, CONF_BUS, CONF_ADDRESS, OPTION_ROLL_OFFSET, OPTION_PITCH_OFFSET, OPTION_TARGET_INTERVAL
from .MPUConstants import MPUConstants

_LOGGER = logging.getLogger(__name__)


class OptionsFlowHandler(config_entries.OptionsFlow):

    def __init__(self, config_entry: config_entries.ConfigEntry) -> None:
        """Initialize options flow."""
        self.config_entry = config_entry

    async def async_step_init(
        self, user_input: dict[str, Any] | None = None
    ) -> ConfigFlowResult:
        if user_input is not None:
            return self.async_create_entry(title="", data=user_input)

        return self.async_show_form(
            step_id="init",
            data_schema=vol.Schema({
                vol.Required(
                    OPTION_ROLL_OFFSET,
                    default=self.config_entry.options.get(OPTION_ROLL_OFFSET, 0.0)
                ): vol.All(vol.Coerce(float), vol.Range(min=-180.0, max=180.0)),
                vol.Required(
                    OPTION_PITCH_OFFSET,
                    default=self.config_entry.options.get(OPTION_PITCH_OFFSET, 0.0)
                ): vol.All(vol.Coerce(float), vol.Range(min=-180.0, max=180.0)),
                vol.Required(
                    OPTION_TARGET_INTERVAL,
                    default=self.config_entry.options.get(OPTION_TARGET_INTERVAL, 1.0)
                ): vol.All(vol.Coerce(float), vol.Range(min=0.1, max=10.0)),
            }),
        )

def list_i2c_busses():
    """Return a list of available I2C bus numbers (e.g. [0, 1])."""
    busses = []
    for entry in os.listdir("/dev"):
        if entry.startswith("i2c-"):
            try:
                busses.append(int(entry.split("-")[1]))
            except ValueError:
                pass
    return sorted(busses)

def list_i2c_devices(bus_id, addr_range=range(0x03, 0x78)):
    """Return a list of I2C device addresses on a given bus."""
    devices = []
    with SMBus(bus_id) as bus:
        for addr in addr_range:  # valid 7-bit addresses
            try:
                bus.write_quick(addr)
                devices.append(addr)
            except OSError:
                pass
    return devices


class MPU6050ConfigFlow(config_entries.ConfigFlow, domain=DOMAIN):
    """Handle a config flow for MPU6050."""

    VERSION = 1
    async def async_step_user(
        self, user_input: dict[str, Any] | None = None
    ) -> FlowResult:
        """Handle the initial step."""
        errors = {}
        busses = await self.hass.async_add_executor_job(
            list_i2c_busses
        )
        _LOGGER.debug("Detected I2C busses: %s", busses)


        # Get all (bus, address) pairs for detected I2C devices
        bus_addresses = [
            (bus, addr)
            for bus in busses
            for addr in await self.hass.async_add_executor_job(list_i2c_devices, bus,[MPUConstants.MPU6050_ADDRESS_AD0_LOW, MPUConstants.MPU6050_ADDRESS_AD0_HIGH])
        ]

        _LOGGER.debug("Detected I2C devices: %s", bus_addresses)
        
        if user_input is not None:
            try:

                bus_str, addr_str = user_input[CONF_BUS_ADDRESS].split(",")
                return self.async_create_entry(
                    title=f"I2C Bus {bus_str}, Address 0x{int(addr_str):02X}",
                    data={
                        CONF_BUS: int(bus_str),
                        CONF_ADDRESS: int(addr_str),
                    },
                    options={
                        OPTION_ROLL_OFFSET: 0.0,
                        OPTION_PITCH_OFFSET: 0.0,
                    }
                )
            except CannotConnect:
                errors["base"] = "cannot_connect"
            except Exception:  # pylint: disable=broad-except
                _LOGGER.exception("Unexpected exception")
                errors["base"] = "unknown"

        return self.async_show_form(
            step_id="user",
            data_schema=vol.Schema(
                {
                    vol.Required(CONF_BUS_ADDRESS): selector.SelectSelector(
                        selector.SelectSelectorConfig(
                            options=[
                                selector.SelectOptionDict(
                                    value=f"{bus},{addr}",
                                    label=f"I2C Bus {bus}, Address 0x{addr:02X}"
                                )
                                for bus, addr in bus_addresses
                            ],
                            mode=selector.SelectSelectorMode.DROPDOWN,
                        )
                    ),
                }
            ),
            errors=errors,
        )

    @staticmethod
    @callback
    def async_get_options_flow(
        config_entry: ConfigEntry,
    ) -> OptionsFlowHandler:
        """Get the options flow for this handler."""
        return OptionsFlowHandler(config_entry)


class CannotConnect(HomeAssistantError):
    """Error to indicate we cannot connect."""
