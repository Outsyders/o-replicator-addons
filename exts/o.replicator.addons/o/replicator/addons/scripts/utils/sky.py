import carb
import carb.input
import omni.usd
import carb.tokens
import carb.settings
import omni.client
import omni.kit.app
import omni.kit.test

from omni.kit.environment.core import SkyHelper, SkyType, EnvironmentSettings, SunstudyPlayer, get_sunstudy_player


def load_sky(sky_name: str = "ClearSky") -> SunstudyPlayer:
    SKIES_PATH = "https://omniverse-content-production.s3.us-west-2.amazonaws.com/Assets/Skies/2022_1/Skies/"

    try:
        from omni.kit.actions.core import get_action_registry

        __action_registry = get_action_registry()
        __action = __action_registry.get_action("omni.kit.environment.core", "import")
    except ImportError:
        __action = None
        __action_registry = None

    def __execute_action(sky_type, sky_url):
        if __action:
            __action.execute(sky_type, sky_url)
        else:
            from omni.kit.environment.core import import_environment

            import_environment(sky_type, sky_url)

    sky_url = f"{SKIES_PATH}/Dynamic/{sky_name}.usd"
    sky_type = SkyHelper.get_env_file_type(sky_url)

    __execute_action(sky_type, sky_url)

    # Wait for sky loaded
    # await asyncio.sleep(20)

    # await self._capture_viewport_and_compare("dynamic")

    (exist_sky_path, exist_asset_path) = SkyHelper.find_sky()

    print(exist_sky_path + "/Looks/SkyMaterial", exist_asset_path)

    sunstudy_player = get_sunstudy_player()

    return sunstudy_player


# sunstudy_player.latitude = 25.786673
# sunstudy_player.longitude = -80.127817

# # in hours
# sunstudy_player.current_time = 6
