struct s_placer_opts;
struct s_annealing_sched;
struct pcontext;

void *parallel_place (void *);
void free_context(struct pcontext *context);
void restore_context(struct pcontext *context, float *cost, float *bb_cost, float *timing_cost, float *delay_cost, float *rlim, int *pins_on_block, float *net_cost, float *temp_net_cost);
void alloc_context (struct pcontext *context, float update_freq,
	float inverse_prev_bb_cost, float inverse_prev_timing_cost,
	struct s_annealing_sched *annealing_sched, struct s_placer_opts *placer_opts,
	float *net_cost, float *temp_net_cost, float update_freq, float cost,
	float bb_cost, float timing_cost, float delay_cost, float rlim);
void copy_context (struct pcontext *, struct pcontext *, int);
void merge_contexts (void *ptr);
