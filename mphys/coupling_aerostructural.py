import openmdao.api as om
from .coupling_group import CouplingGroup
from .geo_disp import GeoDisp


class CouplingAeroStructural(CouplingGroup):
    """
    The standard aerostructural coupling problem.
    """

    def initialize(self):
        self.options.declare('aero_builder', recordable=False)
        self.options.declare('struct_builder', recordable=False)
        self.options.declare('ldxfer_builder', recordable=False)
        self.options.declare('balance_group', recordable=False,default=None)
        self.options.declare("scenario_name", recordable=True, default=None)

    def setup(self):
        aero_builder = self.options['aero_builder']
        struct_builder = self.options['struct_builder']
        ldxfer_builder = self.options['ldxfer_builder']
        balance_group = self.options['balance_group']
        scenario_name = self.options['scenario_name']

        disp_xfer, load_xfer = ldxfer_builder.get_coupling_group_subsystem(scenario_name)
        aero = aero_builder.get_coupling_group_subsystem(scenario_name)
        struct = struct_builder.get_coupling_group_subsystem(scenario_name)

        geo_disp = GeoDisp(number_of_nodes=aero_builder.get_number_of_nodes())

        self.mphys_add_subsystem('disp_xfer', disp_xfer)
        self.mphys_add_subsystem('geo_disp', geo_disp)
        # if balance_group is not None:
        #     aero_post = aero_builder.get_post_coupling_subsystem_schur(self.name)
        #     self.mphys_add_subsystem(
        #             "coupling_schur",
        #             CouplingAeroSchur(
        #                 # coupling_group=coupling_schur,
        #                 # aero_pre=aero_pre,
        #                 coupling=aero,
        #                 aero_post=aero_post,
        #                 balance_group=balance_group,
        #             ),
        #         )
        # else:
        self.mphys_add_subsystem('aero', aero)
        self.mphys_add_subsystem('load_xfer', load_xfer)
        self.mphys_add_subsystem('struct', struct)

        self.nonlinear_solver = om.NonlinearBlockGS(maxiter=25, iprint=2,
                                                    atol=1e-8, rtol=1e-8,
                                                    use_aitken=True)
        self.linear_solver = om.LinearBlockGS(maxiter=25, iprint=2,
                                              atol=1e-8, rtol=1e-8,
                                              use_aitken=True)



class CouplingAeroSchur(CouplingGroup):
    """
    The standard aerostructural coupling problem.
    """

    def initialize(self):
        # self.options.declare("aero_pre", recordable=False, default=None)
        self.options.declare("coupling", recordable=False, default=None)
        self.options.declare("aero_post", recordable=False, default=None)
        self.options.declare("balance_group", recordable=False, default=None)

    def setup(self):
        # aero_pre = self.options["aero_pre"]
        coupling = self.options["coupling"]
        aero_post = self.options["aero_post"]
        balance_group = self.options["balance_group"]

        coupling_group = CouplingAeroTopSchur(coupling=coupling, aero_post=aero_post)

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
        # self.options.declare("aero_pre", recordable=False, default=None)
        self.options.declare("coupling", recordable=False, default=None)
        self.options.declare("aero_post", recordable=False, default=None)

    def setup(self):
        # aero_pre = self.options["aero_pre"]
        coupling = self.options["coupling"]
        aero_post = self.options["aero_post"]

        # self.mphys_add_subsystem("aero_pre", aero_pre)
        self.mphys_add_subsystem("aero", coupling)
        self.mphys_add_subsystem("aero_post", aero_post)

        self.nonlinear_solver = om.NonlinearRunOnce()
        self.linear_solver = om.LinearRunOnce()