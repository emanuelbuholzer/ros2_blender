import addon_utils
import bpy
from bpy.props import IntProperty


class AddonMinimalReloadOperator(bpy.types.Operator):
    bl_idname = "addon_minimal.restart_and_reload_preferences"
    bl_label = "Restart addon_minimal and reload the preferences"

    def execute(self, context):
        addon_utils.disable("addon_minimal")
        addon_utils.enable("addon_minimal")
        return {"FINISHED"}


class AddonMinimalPreferences(bpy.types.AddonPreferences):
    bl_idname = "addon_minimal"

    domain_id: IntProperty(
        name="ROS 2 domain ID",  # noqa: F722
        description="Safe between 0 and 101, inclusive, but possible up to 232",  # noqa: F722
        default=0,
        min=0,
        soft_max=101,
        max=232,
    )

    def draw(self, context):
        self.layout.prop(self, "domain_id")
        self.layout.operator("addon_minimal.restart_and_reload_preferences")


def register():
    bpy.utils.register_class(AddonMinimalReloadOperator)
    bpy.utils.register_class(AddonMinimalPreferences)


def unregister():
    bpy.utils.unregister_class(AddonMinimalPreferences)
    bpy.utils.unregister_class(AddonMinimalReloadOperator)
