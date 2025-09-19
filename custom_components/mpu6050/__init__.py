import logging

from homeassistant.config_entries import ConfigEntry
from homeassistant.core import HomeAssistant
from homeassistant.const import Platform

from .MPU6050 import MPU6050Device

_LOGGER = logging.getLogger(__name__)
_LOGGER.info("MPU6050 integration is initializing")

PLATFORMS = [Platform.SENSOR, Platform.SWITCH]

async def async_setup_entry(hass: HomeAssistant, entry: ConfigEntry) -> bool:
    """Set up the MPU6050 integration from a config entry."""
    entry["device"] = MPU6050Device(
        entry
    )
    await hass.config_entries.async_forward_entry_setups(entry, PLATFORMS)
    return True


async def async_unload_entry(hass: HomeAssistant, entry: ConfigEntry) -> bool:
    """Unload a config entry."""
    unload_ok = await hass.config_entries.async_unload_platforms(entry, PLATFORMS)
    if unload_ok:
        hass.data["mpu6050"].pop(entry.entry_id, None)
    return unload_ok