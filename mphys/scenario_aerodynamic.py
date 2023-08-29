from .scenario import Scenario
from .coupling_group import CouplingGroup
import openmdao.api as om


class ScenarioAerodynamic(Scenario):
    def initialize(self):
        """
        A class to perform a single discipline aerodynamic case.
        The Scenario will add the aerodynamic builder's precoupling subsystem,
        the coupling subsystem, and the postcoupling subsystem.
        """
        super().initialize()

        self.options.declare("aero_builder", recordable=False, desc="The MPhys builder for the aerodynamic solver")
        self.options.declare(
            "in_MultipointParallel",
            default=False,
            types=bool,
            desc="Set to `True` if adding this scenario inside a MultipointParallel Group.",
        )
        # self.options.declare(
        #     "add_SchurCoupling",
        #     default=False,
        #     types=bool,
        #     desc="Set to `True` if adding this scenario inside a MultipointParallel Group.",
        # )
        self.options.declare(
            "geometry_builder", default=None, recordable=False, desc="The optional MPhys builder for the geometry"
        )
        self.options.declare(
            "balance_group", default=None, recordable=False, desc="The optional MPhys builder for the geometry"
        )

    def _mphys_scenario_setup(self):
        aero_builder = self.options["aero_builder"]
        geometry_builder = self.options["geometry_builder"]
        balance_group = geometry_builder = self.options["balance_group"]

        if balance_group is None:
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
        else:
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
            # coupling_schur = self.mphys_add_subsystem("coupling", CouplingGroup())
            # coupling_schur.mphys_add_subsystem("aero_pre", aero_builder.get_pre_coupling_subsystem(self.name))
            aero_pre = aero_builder.get_pre_coupling_subsystem(self.name)
            # self._mphys_add_pre_coupling_subsystem_from_builder("aero", aero_builder, self.name)
            # coupling_schur.mphys_add_subsystem("coupling", aero_builder.get_coupling_group_subsystem(self.name))
            coupling = aero_builder.get_coupling_group_subsystem(self.name)
            # coupling_schur.mphys_add_subsystem("aero_post", aero_builder.get_post_coupling_subsystem(self.name))
            aero_post = aero_builder.get_post_coupling_subsystem(self.name)
            self.mphys_add_subsystem(
                "coupling_schur",
                CouplingAeroSchur(
                    # coupling_group=coupling_schur,
                    aero_pre=aero_pre,
                    coupling=coupling,
                    aero_post=aero_post,
                    balance_group=balance_group,
                ),
            )
            # self._mphys_add_post_coupling_subsystem_from_builder("aero", aero_builder, self.name)


class CouplingAeroSchur(CouplingGroup):
    """
    The standard aerostructural coupling problem.
    """

    def initialize(self):
        self.options.declare("aero_pre", recordable=False, default=None)
        self.options.declare("coupling", recordable=False, default=None)
        self.options.declare("aero_post", recordable=False, default=None)
        self.options.declare("balance_group", recordable=False, default=None)

    def setup(self):
        aero_pre = self.options["aero_pre"]
        coupling = self.options["coupling"]
        aero_post = self.options["aero_post"]
        balance_group = self.options["balance_group"]

        coupling_group = CouplingAeroTopSchur(aero_pre=aero_pre, coupling=coupling, aero_post=aero_post)

        self.mphys_add_subsystem("coupling_group", coupling_group)
        self.mphys_add_subsystem("balance_group", balance_group)

        self.nonlinear_solver = om.NonlinearSchurSolver(
            atol=1e-2,
            rtol=1e-20,
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


class CouplingAeroTopSchur(CouplingGroup):
    """
    The standard aerostructural coupling problem.
    """

    def initialize(self):
        self.options.declare("aero_pre", recordable=False, default=None)
        self.options.declare("coupling", recordable=False, default=None)
        self.options.declare("aero_post", recordable=False, default=None)

    def setup(self):
        aero_pre = self.options["aero_pre"]
        coupling = self.options["coupling"]
        aero_post = self.options["aero_post"]

        self.mphys_add_subsystem("aero_pre", aero_pre)
        self.mphys_add_subsystem("coupling", coupling)
        self.mphys_add_subsystem("aero_post", aero_post)

        self.nonlinear_solver = om.NonlinearRunOnce()
        self.linear_solver = om.LinearRunOnce()
