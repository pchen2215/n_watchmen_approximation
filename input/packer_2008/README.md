Packer 2008 polygon set

Source:
- Eli Packer, "Computing Multiple Watchman Routes," WEA 2008 / LNCS 5038.
- Public PDF mirror: https://people.scs.carleton.ca/~kranakis/conferences/ips-papers/MR-mobile_robots.pdf

Notes:
- The paper figures show the polygons visually, but the coordinate files do not appear to be published with the paper.
- The files in `simple/` are hand-traced CCW approximations of the simple polygons shown in Figure 7.
- The files in `with_holes/` are hand-traced approximations of the polygons-with-holes shown in Figure 7.
- `with_holes/` uses a future-friendly multi-ring format:
  - ring 1 is the outer boundary in CCW order
  - each later ring is a hole in CW order
  - rings are separated by a blank line
- The current `n_watchmen_approximation.cpp` loader reads only a single ring, so the `simple/` files are directly compatible today and the `with_holes/` files are stored for reference / future parser support.

Figure 7 mapping:
- `fig7_a_square_with_triangle_hole_rings.txt`
- `fig7_b_spike_box_simple_ccw.txt`
- `fig7_c_irregular_bay_simple_ccw.txt`
- `fig7_d_orthogonal_corridor_simple_ccw.txt`
- `fig7_e_spike_skyline_simple_ccw.txt`
- `fig7_f_random_nonconvex_simple_ccw.txt`
- `fig7_g_wiggle_simple_ccw.txt`
- `fig7_h_square_with_slots_rings.txt`
- `fig7_i_square_teeth_simple_ccw.txt`
- `fig7_j_pinwheel_simple_ccw.txt`
- `fig7_k_spiral_simple_ccw.txt`
- `fig7_l_branching_simple_ccw.txt`
- `fig7_m_grid_holes_rings.txt`
- `fig7_n_square_with_delta_hole_rings.txt`
- `fig7_o_staircase_with_holes_rings.txt`
- `fig7_p_spike_box_sparse_simple_ccw.txt`
