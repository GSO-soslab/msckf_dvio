void ImuInitializer::checkInitGiven() {
  if(param_init.init_given){
    time_I = param_init.init_state(0);
    q_I_G = param_init.init_state.segment(1,4);
    p_G_I = param_init.init_state.segment(5,3);
    v_G_I = param_init.init_state.segment(8,3);
    bg_avg = param_init.init_state.segment(11,3);
    ba_avg = param_init.init_state.segment(14,3);
    
    time_D = param_init.init_state(0);
    time_I_D = 0;

    printf("Initialization result at:\n"
        " IMU time:%f, DVL time:%f, time_I_D:%f\n"
        " q_I_G(xyzw):%f,%f,%f,%f\n"
        " p_G_I:%f,%f,%f\n"
        " v_G_I:%f,%f,%f\n"
        " bg:%f,%f,%f\n"
        " ba:%f,%f,%f\n",
        time_I, time_D, time_I_D,
        q_I_G(0),q_I_G(1),q_I_G(2),q_I_G(3),
        p_G_I(0),p_G_I(1),p_G_I(2),
        v_G_I.x(),v_G_I.y(),v_G_I.z(),
        bg_avg.x(),bg_avg.y(),bg_avg.z(),
        ba_avg.x(),ba_avg.y(),ba_avg.z()
      );

    is_initialized = true;

    cleanBuffer();

  }
}