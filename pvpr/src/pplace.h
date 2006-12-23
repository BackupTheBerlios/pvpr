#define BUFSIZE 300
#define RAND_BUF_SIZE 32

struct pcontext {
	struct s_clb **clb;
//	struct s_net *net;
	struct s_block *block;
	
	struct s_bb *bb_coord_new;
	struct s_bb *bb_edge_new;
	
	int *nets_to_update, *net_block_moved;
	
	unsigned int rand_buf[RAND_BUF_SIZE];
	int first_rand;
	
	struct s_bb *bb_coords, *bb_num_on_edges;
	
	float *net_cost, *temp_net_cost;
	
	char msg[BUFSIZE];
	
	int tot_iter, success_sum, move_lim, moves_since_cost_recompute;
	
	float t, success_rat, rlim;
	float cost, timing_cost, bb_cost, delay_cost;
	float new_bb_cost, new_timing_cost;
	
	float new_delay_cost, place_delay_value;
	float oldt;

	float inverse_prev_bb_cost, inverse_prev_timing_cost;
	
	double av_cost, av_bb_cost, av_timing_cost, av_delay_cost, sum_of_squares, std_dev;

	boolean fixed_pins;  /* Can pads move or not? */
	
	struct s_annealing_sched *annealing_sched;
	
	float update_freq;
	
	int *pins_on_block;
	
	int *duplicate_pins;
	
	int **unique_pin_list;
	
	struct s_placer_opts *placer_opts;
	
	int inet, outer_crit_iter_count, inner_crit_iter_count, inner_recompute_limit;
};

void *parallel_place (void *);
void free_context(struct pcontext *context);
void restore_context(struct pcontext *context, float *cost, float *bb_cost, float *timing_cost, float *delay_cost, float *rlim, int *pins_on_block, float *net_cost, float *temp_net_cost);
void alloc_context (struct pcontext *context, float update_freq,
	float inverse_prev_bb_cost, float inverse_prev_timing_cost,
	struct s_annealing_sched *annealing_sched, struct s_placer_opts *placer_opts,
	float *net_cost, float *temp_net_cost, float cost, float bb_cost,
	float timing_cost, float delay_cost, float rlim, int *duplicate_pins, int **unique_pin_list);
void copy_context (struct pcontext *, struct pcontext *, int);
void merge_contexts (void *ptr);

void fill_rand_buf (unsigned int rand_buf[], int *first_rand);
int p_my_irand (struct pcontext *context, int imax);
float p_my_frand (struct pcontext *context);
