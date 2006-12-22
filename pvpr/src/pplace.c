#include "place.h"
#include "pplace.h"
#include "globals.h"

void *parallel_place (void *arg) {
	int inner_iter;
	struct pcontext *context = (struct pcontext *) arg;
	
	
	while (exit_crit(context->t, context->cost, *annealing_sched) == 0) {
		context->av_cost = 0.;
		context->av_bb_cost = 0.;
		context->av_delay_cost = 0.;
		context->av_timing_cost = 0.;
		context->sum_of_squares = 0.;
		context->success_sum = 0;

		context->inner_crit_iter_count = 1;

		for (inner_iter=0; inner_iter < context->move_lim; inner_iter++) {
			if (ptry_swap(context->t, &(context->cost), &(context->bb_cost),
					&(context->timing_cost), context->rlim, context->pins_on_block,
					context->placer_opts->place_cost_type, NULL, NULL, 0,
					context->fixed_pins, context->placer_opts->place_algorithm, 
					context->placer_opts->timing_tradeoff, context->inverse_prev_bb_cost, 
					context->inverse_prev_timing_cost, &(context->delay_cost)) == 1) {
				context->success_sum++;
				context->av_cost += context->cost;
				context->av_bb_cost += context->bb_cost;
				context->av_timing_cost += context->timing_cost;
				context->av_delay_cost += context->delay_cost;
				context->sum_of_squares += context->cost * context->cost;
			}
		}

/* Lines below prevent too much round-off error from accumulating *
 * in the cost over many iterations.  This round-off can lead to  *
 * error checks failing because the cost is different from what   *
 * you get when you recompute from scratch.                       */
 
		context->moves_since_cost_recompute += context->move_lim;
		if (context->moves_since_cost_recompute > MAX_MOVES_BEFORE_RECOMPUTE) {
			/***HERE***/
			new_bb_cost = recompute_bb_cost (placer_opts.place_cost_type, 
                     placer_opts.num_regions);       
       if (fabs(new_bb_cost - bb_cost) > bb_cost * ERROR_TOL) {
          printf("Error in try_place:  new_bb_cost = %g, old bb_cost = %g.\n",
              new_bb_cost, bb_cost);
          exit (1);
       }
       bb_cost = new_bb_cost;

       if (placer_opts.place_algorithm ==BOUNDING_BOX_PLACE) {
	 cost = new_bb_cost;
       }
       moves_since_cost_recompute = 0;
    }

    tot_iter += move_lim;
    success_rat = ((float) success_sum)/ move_lim;
    if (success_sum == 0) {
       av_cost = cost;
       av_bb_cost = bb_cost;
       av_timing_cost = timing_cost;
       av_delay_cost = delay_cost;
    }
    else {
       av_cost /= success_sum;
       av_bb_cost /= success_sum;
       av_timing_cost /= success_sum;
       av_delay_cost /= success_sum;
    }
    std_dev = get_std_dev (success_sum, sum_of_squares, av_cost);

#ifndef SPEC
    printf("%11.5g  %10.6g %11.6g  %11.6g  %11.6g %11.6g %11.4g %9.4g %8.3g  %7.4g  %7.4g  %10d  ",t, av_cost, 
	   av_bb_cost, av_timing_cost, av_delay_cost, place_delay_value, d_max, success_rat, std_dev, 
	   rlim, crit_exponent,tot_iter);
#endif

    oldt = t;  /* for finding and printing alpha. */
    update_t (&t, std_dev, rlim, success_rat, annealing_sched);

#ifndef SPEC
    printf("%7.4g\n",t/oldt);
#endif

    sprintf(msg,"Cost: %g  BB Cost %g  TD Cost %g  Temperature: %g  d_max: %g",cost, 
	    bb_cost, timing_cost, t, d_max);
    update_screen(MINOR, msg, PLACEMENT, FALSE);
    update_rlim (&rlim, success_rat);

#ifdef VERBOSE 
 dump_clbs();
#endif
 }
}

void copy_context (struct pcontext *arr, struct pcontext context, int n) {
	int cont, i, j, k;
	
 for (cont=0; cont<n; cont++) {
		for (i=0; i<num_blocks; i++) {
			arr[cont].block[i].type = context.block[i].type;
			arr[cont].block[i].x = context.block[i].x;
			arr[cont].block[i].y = context.block[i].y;
		}
	
	
	for (i=0; i<=nx+1; i++) {
		for (j=0; i<=ny+1; j++) {
			arr[cont].clb[i][j].type = context.clb[i][j].type;
			arr[cont].clb[i][j].occ = context.clb[i][j].occ;
			arr[cont].clb[i][j].u.block = context.clb[i][j].u.block;
			for (k=0; k<clb[i][j].occ; k++) {
				arr[cont].clb[i][j].u.io_blocks[k] = context.clb[i][j].u.io_blocks[k];
			}
		}
	}
	
	for (i=0; i<num_nets; i++) {
		arr[cont].bb_coords[i].xmin = context.bb_coords[i].xmin;
		arr[cont].bb_coords[i].xmax = context.bb_coords[i].xmax;	
		arr[cont].bb_coords[i].ymin = context.bb_coords[i].ymin;
		arr[cont].bb_coords[i].ymax = context.bb_coords[i].ymax;	
		arr[cont].net_cost[i] = context.net_cost[i];
		arr[cont].temp_net_cost[i] = context.temp_net_cost[i];
	}

	arr[cont].tot_iter = context.tot_iter;
	arr[cont].success_sum = context.success_sum;
	arr[cont].move_lim = context.move_lim;
	arr[cont].moves_since_cost_recompute = context.moves_since_cost_recompute;
	arr[cont].t = context.t;
	arr[cont].success_rat = context.success_rat;
	arr[cont].rlim = context.rlim;
	arr[cont].cost = context.cost;
	arr[cont].timing_cost = context.timing_cost;
	arr[cont].bb_cost = context.bb_cost;
	arr[cont].new_bb_cost = context.new_bb_cost;
	arr[cont].new_timing_cost = context.new_timing_cost;
	
	arr[cont].delay_cost = context.delay_cost;
	arr[cont].new_delay_cost = context.new_delay_cost;
	arr[cont].place_delay_value = context.place_delay_value;
	arr[cont].oldt = context.oldt;

	arr[cont].av_cost = context.av_cost;
	arr[cont].av_bb_cost = context.av_bb_cost;
	arr[cont].av_timing_cost = context.av_timing_cost;
	arr[cont].av_delay_cost = context.av_delay_cost;
	arr[cont].sum_of_squares = context.sum_of_squares;
	arr[cont].std_dev = context.std_dev;

	arr[cont].fixed_pins = context.fixed_pins;
	
	arr[cont].inet = context.inet;
	arr[cont].outer_crit_iter_count = context.outer_crit_iter_count;
	arr[cont].inner_crit_iter_count = context.inner_crit_iter_count;
	arr[cont].inner_recompute_limit = context.inner_recompute_limit;
	
 }
}

void alloc_context (struct pcontext *context)
{
	int i, j, *index;
	int num_pins;
	
	/*
	context->net = (struct s_net *) my_malloc (num_nets*sizeof(struct s_net));
	assert(context->net);

	for (i=0; i<num_nets; i++) {
		context->net[i].name = (char *) my_malloc (strlen(net[i].name)*sizeof(char));
		assert(context->net[i].name);
		strcpy(context->net[i].name, net[i].name);
		num_pins = net[i].num_pins;
		context->net[i].num_pins = num_pins;
		context->net[i].blocks = (int *) my_malloc (num_pins*sizeof(int));
		assert(context->net[i].blocks);
		context->net[i].blk_pin = (int *) my_malloc (num_pins*sizeof(int));
		assert(context->net[i].blk_pin);
		
		for (j=0; j<num_pins; j++) {
			context->net[i].blocks[j] = net[i].blocks[j];
			context->net[i].blk_pin[j] = net[i].blocks[j];
		}
	}
	*/	
	context->inverse_prev_bb_cost = inverse_prev_bb_cost;
	context->inverse_prev_timing_cost = inverse_prev_timing_cost;

	context->annealing_sched = &annealing_sched;
	context->placer_opts = &placer_opts;
	
	context->block = (struct s_block *) my_malloc (num_blocks*sizeof(struct s_block));
	assert(context->block);
	
	context->pins_on_block = (int *) my_malloc (num_blocks*sizeof(int));
	assert(context->pins_on_block);
	
	for (i=0; i<num_blocks; i++) {
		context->pins_on_block[i] = pins_on_block[i];
		context->block[i].name = (char *) my_malloc (strlen(block[i].name)*sizeof(char));
		assert(context->block[i].name);
		strcpy(context->block[i].name, block[i].name);
		context->block[i].type = block[i].type;
		context->block[i].x = block[i].x;
		context->block[i].y = block[i].y;
		context->block[i].nets = (int *) my_malloc (pins_per_clb * sizeof(int));
		assert(context->block[i].nets);
		for (j=0; j<pins_per_clb; j++) {
			context->block[i].nets[j] = block[i].nets[j];
		}
	}
	
	context->clb = (struct s_clb **) alloc_matrix (0, nx+1, 0, ny+1, sizeof(struct s_clb));
			
	i = 2*io_rat*(nx+ny);
	index = (int *) my_malloc (i*sizeof(int));
	for (i=1;i<=nx;i++) {
		context->clb[i][0].u.io_blocks = index;
		index+=io_rat;
		context->clb[i][ny+1].u.io_blocks = index;
		index+=io_rat;
	}
	for (i=1;i<=ny;i++) {
		context->clb[0][i].u.io_blocks = index;
		index+=io_rat;
		context->clb[nx+1][i].u.io_blocks = index;
		index+=io_rat;
	}
	
	for (i=0; i<=nx+1; i++) {
		for (j=0; i<=ny+1; j++) {
			context->clb[i][j].type = clb[i][j].type;
			context->clb[i][j].occ = clb[i][j].occ;
			context->clb[i][j].u.block = clb[i][j].u.block;
			for (k=0; k<clb[i][j].occ; k++) {
				context->clb[i][j].u.io_blocks[k] = clb[i][j].u.io_blocks[k];
			}
		}
	}

	context->bb_coords = (struct s_bb *) my_malloc (num_nets * sizeof(struct s_bb));
	context->bb_num_on_edges = (struct s_bb *) my_malloc (num_nets * sizeof(struct s_bb));
	
	for (i=0; i<num_nets; i++) {
		context->bb_coords[i].xmin = bb_coords[i].xmin;
		context->bb_coords[i].xmax = bb_coords[i].xmax;	
		context->bb_coords[i].ymin = bb_coords[i].ymin;
		context->bb_coords[i].ymax = bb_coords[i].ymax;	
	}
	
	context->net_cost = (float *) my_malloc (num_nets * sizeof (float));
	context->temp_net_cost = (float *) my_malloc (num_nets * sizeof (float));
	
	for (i=0; i<num_nets; i++) {
		context->net_cost[i] = net_cost[i];
		context->temp_net_cost[i] = temp_net_cost[i];
	}

}
