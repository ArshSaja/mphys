import openmdao.api as om
from .scenario import Scenario
from .coupling_group import CouplingGroup
from .coupling_aerostructural import CouplingAeroStructural


class ScenarioAeroStructural(Scenario):
    def initialize(self):
        """
        A class to perform a single discipline aerodynamic case.
        The Scenario will add the aerodynamic builder's precoupling subsystem,
        the coupling subsystem, and the postcoupling subsystem.
        """
        super().initialize()

        self.options.declare(
            "aero_builder",
            recordable=False,
            desc="The MPhys builder for the aerodynamic solver",
        )
        self.options.declare(
            "struct_builder",
            recordable=False,
            desc="The MPhys builder for the structural solver",
        )
        self.options.declare(
            "ldxfer_builder",
            recordable=False,
            desc="The MPhys builder for the load and displacement transfer",
        )
        self.options.declare(
            "in_MultipointParallel",
            default=False,
            desc="Set to `True` if adding this scenario inside a MultipointParallel Group.",
        )
        self.options.declare(
            "geometry_builder",
            default=None,
            recordable=False,
            desc="The optional MPhys builder for the geometry",
        )
        self.options.declare(
            "balance_group", default=None, recordable=False, desc="The optional MPhys builder for the geometry"
        )
        self.options.declare(
            "coupling_group_type",
            default="full_coupling",
            desc='Limited flexibility for coupling group type to accomodate flutter about jig shape or DLM where coupling group can be skipped: ["full_coupling", "aerodynamics_only", None]',
        )

    def _mphys_scenario_setup(self):
        aero_builder = self.options["aero_builder"]
        struct_builder = self.options["struct_builder"]
        ldxfer_builder = self.options["ldxfer_builder"]
        geometry_builder = self.options["geometry_builder"]
        balance_group = self.options["balance_group"]

        if self.options["in_MultipointParallel"]:
            self._mphys_initialize_builders(aero_builder, struct_builder, ldxfer_builder, geometry_builder)
            self._mphys_add_mesh_and_geometry_subsystems(aero_builder, struct_builder, geometry_builder)

        if balance_group is None:
            self._mphys_add_pre_coupling_subsystem_from_builder("aero", aero_builder, self.name)
            self._mphys_add_pre_coupling_subsystem_from_builder("struct", struct_builder, self.name)
            self._mphys_add_pre_coupling_subsystem_from_builder("ldxfer", ldxfer_builder, self.name)

            if self.options["coupling_group_type"] == "full_coupling":
                coupling_group = CouplingAeroStructural(
                    aero_builder=aero_builder,
                    struct_builder=struct_builder,
                    ldxfer_builder=ldxfer_builder,
                    scenario_name=self.name,
                )
                self.mphys_add_subsystem("coupling", coupling_group)

            elif self.options["coupling_group_type"] == "aerodynamics_only":
                aero = aero_builder.get_coupling_group_subsystem(self.name)
                self.mphys_add_subsystem("aero", aero)

            self._mphys_add_post_coupling_subsystem_from_builder("ldxfer", ldxfer_builder, self.name)
            self._mphys_add_post_coupling_subsystem_from_builder("aero", aero_builder, self.name)
            self._mphys_add_post_coupling_subsystem_from_builder("struct", struct_builder, self.name)
        else:
            aero_pre = aero_builder.get_pre_coupling_subsystem(self.name)
            struct_pre = struct_builder.get_pre_coupling_subsystem(self.name)
            ldxfer_pre = ldxfer_builder.get_pre_coupling_subsystem(self.name)

            if self.options["coupling_group_type"] == "full_coupling":
                coupling = CouplingAeroStructural(
                    aero_builder=aero_builder,
                    struct_builder=struct_builder,
                    ldxfer_builder=ldxfer_builder,
                    scenario_name=self.name,
                )

            elif self.options["coupling_group_type"] == "aerodynamics_only":
                coupling = aero_builder.get_coupling_group_subsystem(self.name)

            aero_post = aero_builder.get_post_coupling_subsystem(self.name)
            struct_post = struct_builder.get_post_coupling_subsystem(self.name)
            ldxfer_post = ldxfer_builder.get_post_coupling_subsystem(self.name)

            self.mphys_add_subsystem(
                "coupling_aerostruct",
                CouplingAeroStructSchur(
                    aero_pre=aero_pre,
                    struct_pre=struct_pre,
                    ldxfer_pre=ldxfer_pre,
                    coupling=coupling,
                    aero_post=aero_post,
                    struct_post=struct_post,
                    ldxfer_post=ldxfer_post,
                    balance_group=balance_group,
                    coupling_group_type=self.options["coupling_group_type"],
                ),
            )

    def _mphys_initialize_builders(self, aero_builder, struct_builder, ldxfer_builder, geometry_builder):
        aero_builder.initialize(self.comm)
        struct_builder.initialize(self.comm)
        ldxfer_builder.initialize(self.comm)
        if geometry_builder is not None:
            geometry_builder.initialize(self.comm)

    def _mphys_add_mesh_and_geometry_subsystems(self, aero_builder, struct_builder, geometry_builder):
        if geometry_builder is None:
            self.mphys_add_subsystem("aero_mesh", aero_builder.get_mesh_coordinate_subsystem(self.name))
            self.mphys_add_subsystem("struct_mesh", struct_builder.get_mesh_coordinate_subsystem(self.name))
        else:
            self.add_subsystem("aero_mesh", aero_builder.get_mesh_coordinate_subsystem(self.name))
            self.add_subsystem("struct_mesh", struct_builder.get_mesh_coordinate_subsystem(self.name))
            self.mphys_add_subsystem("geometry", geometry_builder.get_mesh_coordinate_subsystem(self.name))
            self.connect("aero_mesh.x_aero0", "geometry.x_aero_in")
            self.connect("struct_mesh.x_struct0", "geometry.x_struct_in")


class CouplingAeroStructSchur(CouplingGroup):
    """
    The standard aerostructural coupling problem for schur.
    """

    def initialize(self):
        self.options.declare("aero_pre", recordable=False, default=None)
        self.options.declare("struct_pre", recordable=False, default=None)
        self.options.declare("ldxfer_pre", recordable=False, default=None)
        self.options.declare("coupling", recordable=False, default=None)
        self.options.declare("aero_post", recordable=False, default=None)
        self.options.declare("struct_post", recordable=False, default=None)
        self.options.declare("ldxfer_post", recordable=False, default=None)
        self.options.declare("balance_group", recordable=False, default=None)
        self.options.declare("coupling_group_type", default=None)

    def setup(self):
        aero_pre = self.options["aero_pre"]
        struct_pre = self.options["struct_pre"]
        ldxfer_pre = self.options["ldxfer_pre"]
        coupling = self.options["coupling"]
        aero_post = self.options["aero_post"]
        struct_post = self.options["struct_post"]
        ldxfer_post = self.options["ldxfer_post"]
        balance_group = self.options["balance_group"]

        coupling_group = CouplingAeroStructTopSchur(
            aero_pre=aero_pre,
            struct_pre=struct_pre,
            ldxfer_pre=ldxfer_pre,
            coupling=coupling,
            aero_post=aero_post,
            struct_post=struct_post,
            ldxfer_post=ldxfer_post,
            coupling_group_type=self.options["coupling_group_type"],
        )

        self.mphys_add_subsystem("coupling_group", coupling_group)
        self.mphys_add_subsystem("balance_group", balance_group)

        self.nonlinear_solver = om.NonlinearSchurSolver(
            atol=1e-8,
            rtol=1e-8,
            solve_subsystems=True,
            maxiter=10,
            max_sub_solves=60,
            err_on_non_converge=True,
            mode_nonlinear="rev",
            groupNames=["coupling_group", "balance_group"],
        )
        self.linear_solver = om.LinearSchur(
            mode_linear="rev",
            groupNames=["coupling_group", "balance_group"],
        )
        self.set_solver_print(level=2, depth=4)


class CouplingAeroStructTopSchur(CouplingGroup):
    """
    The layer which constrains all the components of aerostructural.
    """

    def initialize(self):
        self.options.declare("aero_pre", recordable=False, default=None)
        self.options.declare("struct_pre", recordable=False, default=None)
        self.options.declare("ldxfer_pre", recordable=False, default=None)
        self.options.declare("coupling", recordable=False, default=None)
        self.options.declare("aero_post", recordable=False, default=None)
        self.options.declare("struct_post", recordable=False, default=None)
        self.options.declare("ldxfer_post", recordable=False, default=None)
        self.options.declare("coupling_group_type", default=None)

    def setup(self):
        aero_pre = self.options["aero_pre"]
        struct_pre = self.options["struct_pre"]
        ldxfer_pre = self.options["ldxfer_pre"]
        coupling = self.options["coupling"]
        aero_post = self.options["aero_post"]
        struct_post = self.options["struct_post"]
        ldxfer_post = self.options["ldxfer_post"]

        if aero_pre is not None:
            self.mphys_add_subsystem("aero_pre", aero_pre)
        if struct_pre is not None:
            self.mphys_add_subsystem("struct_pre", struct_pre)
        if ldxfer_pre is not None:
            self.mphys_add_subsystem("ldxfer_pre", ldxfer_pre)

        if self.options["coupling_group_type"] == "full_coupling":
            self.mphys_add_subsystem("coupling", coupling)
        elif self.options["coupling_group_type"] == "aerodynamics_only":
            self.mphys_add_subsystem("aero", coupling)

        if ldxfer_post is not None:
            self.mphys_add_subsystem("ldxfer_post", ldxfer_post)
        if aero_post is not None:
            self.mphys_add_subsystem("aero_post", aero_post)
        if struct_post is not None:
            self.mphys_add_subsystem("struct_post", struct_post)

        self.nonlinear_solver = om.NonlinearRunOnce()
        self.linear_solver = om.LinearRunOnce()
