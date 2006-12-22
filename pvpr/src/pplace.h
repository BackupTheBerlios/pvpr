struct pcontext {
	struct s_clb **clb;
//	struct s_net *net;
	struct s_block *block;
	
	struct s_bb *bb_coords, *bb_num_on_edges;
	
	static float *net_cost, *temp_net_cost;
	
	int tot_iter, success_sum, move_lim, moves_since_cost_recompute;
	
	float t, success_rat, rlim;
	float cost, timing_cost, bb_cost;
	float new_bb_cost, new_timing_cost;
	
	float delay_cost, new_delay_cost,  place_delay_value;
	float oldt;

	float inverse_prev_bb_cost, inverse_prev_timing_cost;
	
	double av_cost, av_bb_cost, av_timing_cost, av_delay_cost, sum_of_squares, std_dev;

	boolean fixed_pins;  /* Can pads move or not? */
	
	struct s_annealing_sched *annealing_sched;
	
	int *pins_on_block;
	
	struct s_placer_opts *placer_opts;
	
	int inet, outer_crit_iter_count, inner_crit_iter_count, inner_recompute_limit;
}

void *parallel_place (void *);
void alloc_context (struct pcontext *);
void copy_context (struct pcontext *, struct pcontext, int);