from .scenario import Scenario


class ScenarioSchur(Scenario):
    def initialize(self):
        """
        A class to perform a single discipline aerodynamic case.
        The Scenario will add the aerodynamic builder's precoupling subsystem,
        the coupling subsystem, and the postcoupling subsystem.
        """
        super().initialize()

        self.options.declare("scenario_builder", recordable=False, desc="The MPhys builder for the aerodynamic solver")
        self.options.declare(
            "in_MultipointParallel",
            default=False,
            types=bool,
            desc="Set to `True` if adding this scenario inside a MultipointParallel Group.",
        )
        self.options.declare(
            "balance_comp", default=None, recordable=False, desc="The optional MPhys builder for the geometry"
        )

    def _mphys_scenario_setup(self):
        scenario_builder = self.options["scenario_builder"]
        aero_builder = scenario_builder.options["aero_builder"]
        geometry_builder = scenario_builder.options["geometry_builder"]
        balance_comp = self.options["balance_comp"]

        if self.options["in_MultipointParallel"]:
            aero_builder.initialize(self.comm)

            if geometry_builder is not None:
                geometry_builder.initialize(self.comm)
                self.add_subsystem("mesh", aero_builder.get_mesh_coordinate_subsystem(self.name))
                self.mphys_add_subsystem("geometry", geometry_builder.get_mesh_coordinate_subsystem(self.name))
                self.connect("mesh.x_aero0", "geometry.x_aero_in")
            else:
                self.mphys_add_subsystem("mesh", aero_builder.get_mesh_coordinate_subsystem(self.name))
            self.connect("x_aero0", "x_aero")

        self._mphys_add_pre_coupling_subsystem_from_builder("aero", aero_builder, self.name)
        self.mphys_add_subsystem("coupling", aero_builder.get_coupling_group_subsystem(self.name))
        self._mphys_add_post_coupling_subsystem_from_builder("aero", aero_builder, self.name)
        self.add_subsystem("balance", balance_comp)
