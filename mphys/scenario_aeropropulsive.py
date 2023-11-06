import openmdao.api as om
from .scenario import Scenario
from .coupling_group import CouplingGroup


class ScenarioAeropropulsive(Scenario):
    def initialize(self):
        """
        A class to perform an aeropropulsive case.
        The Scenario will add the aerodynamic and propulsion builders' precoupling subsystem,
        the coupling subsystem, and the postcoupling subsystem.
        """
        super().initialize()

        self.options.declare("aero_builder", recordable=False, desc="The MPhys builder for the aerodynamic solver")
        self.options.declare("prop_builder", recordable=False, desc="The MPhys builder for the propulsion model")
        self.options.declare(
            "balance_builder", recordable=False, desc="The MPhys builder for the balance group", default=None
        )
        self.options.declare(
            "in_MultipointParallel",
            default=False,
            desc="Set to `True` if adding this scenario inside a MultipointParallel Group.",
        )
        self.options.declare(
            "geometry_builder", default=None, recordable=False, desc="The optional MPhys builder for the geometry"
        )
        self.options.declare(
            "balance_group", default=None, recordable=False, desc="The optional MPhys builder for the geometry"
        )

    def _mphys_scenario_setup(self):
        aero_builder = self.options["aero_builder"]
        prop_builder = self.options["prop_builder"]
        balance_builder = self.options["balance_builder"]
        balance_group = self.options["balance_group"]
        geometry_builder = self.options["geometry_builder"]

        if self.options["in_MultipointParallel"]:
            self._mphys_initialize_builders(aero_builder, prop_builder, geometry_builder)
            self._mphys_add_mesh_and_geometry_subsystems(aero_builder, prop_builder, geometry_builder)

        if balance_group is None:
            self._mphys_add_pre_coupling_subsystem_from_builder("aero", aero_builder, self.name)
            self._mphys_add_pre_coupling_subsystem_from_builder("prop", prop_builder, self.name)

            coupling_group = CouplingAeropropulsive(
                aero_builder=aero_builder, prop_builder=prop_builder, balance_builder=balance_builder, scenario_name=self.name
            )
            self.mphys_add_subsystem("coupling", coupling_group)

            self._mphys_add_post_coupling_subsystem_from_builder("aero", aero_builder, self.name)
            self._mphys_add_post_coupling_subsystem_from_builder("prop", prop_builder, self.name)
        else:
            aero_pre = aero_builder.get_pre_coupling_subsystem(self.name)
            prop_pre = prop_builder.get_pre_coupling_subsystem(self.name)
            # ldxfer_pre = ldxfer_builder.get_pre_coupling_subsystem(self.name)

            coupling = CouplingAeropropulsive(
                aero_builder=aero_builder, prop_builder=prop_builder, balance_builder=balance_builder, scenario_name=self.name
            )

            aero_post = aero_builder.get_post_coupling_subsystem_schur(self.name)
            prop_post = prop_builder.get_post_coupling_subsystem(self.name)

            self.mphys_add_subsystem(
                "coupling_schur",
                CouplingAeroPropSchur(
                    aero_pre=aero_pre,
                    prop_pre=prop_pre,
                    coupling=coupling,
                    aero_post=aero_post,
                    prop_post=prop_post,
                    balance_group=balance_group,
                ),
            )
            self._mphys_add_post_coupling_subsystem_from_builder("aero", aero_builder, self.name)
            self._mphys_add_post_coupling_subsystem_from_builder("prop", prop_builder, self.name)


    def _mphys_initialize_builders(self, aero_builder, prop_builder, geometry_builder):
        aero_builder.initialize(self.comm)
        prop_builder.initialize(self.comm)
        if geometry_builder is not None:
            geometry_builder.initialize(self.comm)

    def _mphys_add_mesh_and_geometry_subsystems(self, aero_builder, prop_builder, geometry_builder):

        if geometry_builder is None:
            self.mphys_add_subsystem("aero_mesh", aero_builder.get_mesh_coordinate_subsystem(self.name))
            # self.mphys_add_subsystem("prop_mesh", prop_builder.get_mesh_coordinate_subsystem(self.name))
        else:
            self.add_subsystem("aero_mesh", aero_builder.get_mesh_coordinate_subsystem(self.name))
            # the propulsion model does not need a mesh with pycycle
            # self.add_subsystem("prop_mesh", prop_builder.get_mesh_coordinate_subsystem(self.name))
            self.mphys_add_subsystem("geometry", geometry_builder.get_mesh_coordinate_subsystem(self.name))
            self.connect("aero_mesh.x_aero0", "geometry.x_aero_in")
            # the propulsion model does not need a mesh with pycycle
            # self.connect("prop_mesh.x_prop0", "geometry.x_prop_in")

    def mphys_make_aeroprop_conn(self, aero2prop_conn, prop2aero_conn):
        # TODO automate this with mphys_result or mphys_coupling tags

        # make the connections
        for k, v in aero2prop_conn.items():
            if self.options["balance_group"] is None:
                self.connect("coupling.aero.%s" % k, "coupling.prop.%s" % v)
            else:
                self.connect("coupling_schur.coupling_group.coupling.aero.%s" % k, "coupling.prop.%s" % v)
        for k, v in prop2aero_conn.items():
            if self.options["balance_group"] is None:
                self.connect("coupling.prop.%s" % k, "coupling.aero.%s" % v)
            else:
                self.connect("coupling_schur.coupling_group.coupling.prop.%s" % k, "coupling.aero.%s" % v)


class CouplingAeropropulsive(CouplingGroup):
    """
    The standard aeropropulsive coupling problem.
    """

    def initialize(self):
        self.options.declare("aero_builder", recordable=False)
        self.options.declare("prop_builder", recordable=False)
        self.options.declare("balance_builder", recordable=False, default=None)
        self.options.declare("scenario_name", recordable=True, default=None)

    def setup(self):
        aero_builder = self.options["aero_builder"]
        prop_builder = self.options["prop_builder"]
        balance_builder = self.options["balance_builder"]
        scenario_name = self.options["scenario_name"]

        aero = aero_builder.get_coupling_group_subsystem(scenario_name)
        prop = prop_builder.get_coupling_group_subsystem(scenario_name)

        self.mphys_add_subsystem("aero", aero)
        self.mphys_add_subsystem("prop", prop)

        if balance_builder is not None:
            balance = balance_builder.get_coupling_group_subsystem(scenario_name)
            self.mphys_add_subsystem("balance", balance)

        self.nonlinear_solver = om.NonlinearBlockGS(maxiter=25, iprint=2, atol=1e-8, rtol=1e-8)
        self.linear_solver = om.LinearBlockGS(maxiter=25, iprint=2, atol=1e-8, rtol=1e-8)


class CouplingAeroPropSchur(CouplingGroup):
    """
    The standard aerostructural coupling problem for schur.
    """

    def initialize(self):
        self.options.declare("aero_pre", recordable=False, default=None)
        self.options.declare("prop_pre", recordable=False, default=None)
        self.options.declare("coupling", recordable=False, default=None)
        self.options.declare("aero_post", recordable=False, default=None)
        self.options.declare("prop_post", recordable=False, default=None)
        self.options.declare("balance_group", recordable=False, default=None)

    def setup(self):
        aero_pre = self.options["aero_pre"]
        prop_pre = self.options["prop_pre"]
        coupling = self.options["coupling"]
        aero_post = self.options["aero_post"]
        prop_post = self.options["prop_post"]
        balance_group = self.options["balance_group"]

        coupling_group = CouplingAeroPropTopSchur(
            aero_pre=aero_pre,
            prop_pre=prop_pre,
            coupling=coupling,
            aero_post=aero_post,
            prop_post=prop_post,
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


class CouplingAeroPropTopSchur(CouplingGroup):
    """
    The layer which constrains all the components of aerostructural.
    """

    def initialize(self):
        self.options.declare("aero_pre", recordable=False, default=None)
        self.options.declare("prop_pre", recordable=False, default=None)
        self.options.declare("coupling", recordable=False, default=None)
        self.options.declare("aero_post", recordable=False, default=None)
        self.options.declare("prop_post", recordable=False, default=None)

    def setup(self):
        aero_pre = self.options["aero_pre"]
        prop_pre = self.options["prop_pre"]
        coupling = self.options["coupling"]
        aero_post = self.options["aero_post"]
        prop_post = self.options["prop_post"]

        if aero_pre is not None:
            self.mphys_add_subsystem("aero_pre", aero_pre)
        if prop_pre is not None:
            self.mphys_add_subsystem("prop_pre", prop_pre)

        self.mphys_add_subsystem("coupling", coupling)

        if aero_post is not None:
            self.mphys_add_subsystem("aero_post", aero_post)
        if prop_post is not None:
            self.mphys_add_subsystem("prop_post", prop_post)

        self.nonlinear_solver = om.NonlinearRunOnce()
        self.linear_solver = om.LinearRunOnce()
