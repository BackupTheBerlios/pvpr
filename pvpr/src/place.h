#include <pthread.h>

static const float cross_count[50] = {   /* [0..49] */
1.0,    1.0,    1.0,    1.0828, 1.1536, 1.2206, 1.2823, 1.3385, 1.3991, 1.4493,
1.4974, 1.5455, 1.5937, 1.6418, 1.6899, 1.7304, 1.7709, 1.8114, 1.8519, 1.8924,
1.9288, 1.9652, 2.0015, 2.0379, 2.0743, 2.1061, 2.1379, 2.1698, 2.2016, 2.2334,
2.2646, 2.2958, 2.3271, 2.3583, 2.3895, 2.4187, 2.4479, 2.4772, 2.5064, 2.5356,
2.5610, 2.5864, 2.6117, 2.6371, 2.6625, 2.6887, 2.7148, 2.7410, 2.7671, 2.7933};

void try_place (struct s_placer_opts placer_opts,struct s_annealing_sched 
                annealing_sched, t_chan_width_dist chan_width_dist, 
		struct s_router_opts router_opts, 
		struct s_det_routing_arch det_routing_arch,
		t_segment_inf *segment_inf,
		t_timing_inf timing_inf,
		t_subblock_data *subblock_data_ptr);

void read_place (char *place_file, char *net_file, char *arch_file,
                 struct s_placer_opts placer_opts, struct s_router_opts router_opts,
		 t_chan_width_dist chan_width_dist, 
		 struct s_det_routing_arch det_routing_arch, 
		 t_segment_inf *segment_inf,
		 t_timing_inf timing_inf, t_subblock_data *subblock_data_ptr);

int exit_crit (float t, float cost, struct s_annealing_sched 
         annealing_sched);
         
double get_std_dev (int n, double sum_x_squared, double av_x);

void update_t (float *t, float std_dev, float rlim, float success_rat,
       struct s_annealing_sched annealing_sched); 

void update_rlim (float *rlim, float success_rat);

float get_net_cost (int inet, struct s_bb *bb_ptr);

