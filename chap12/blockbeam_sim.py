# local (controlbook)
from case_studies import common, E_blockbeam


blockbeam = E_blockbeam.Dynamics()
controller = E_blockbeam.ControllerSSI()

z_ref = common.SignalGenerator(amplitude=0.15, frequency=0.05, y_offset=0.25)
theta_ref = None  #  allows controller outer loop to fill in this value (not necessary)
refs = [z_ref]

time, x_hist, u_hist, r_hist, *_ = common.run_simulation(
    blockbeam,
    refs,
    controller,
    controller_input="state",
    t_final=60,
    dt=E_blockbeam.params.ts,
)

viz = E_blockbeam.Visualizer(time, x_hist, u_hist, r_hist)
viz.plot()
# viz.animate()
