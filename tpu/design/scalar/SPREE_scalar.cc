/******
 * Pipelined Processor
 *
 *
 * pipe3_forwardAB_mulshift_stall_bpred1B_external_div_vec.cc
 *
 * F - R/E - W
 ******/

    // Enable Coprocessor instructions
    MipsOp::enable(MIPSOP_CFC2);
    MipsOp::enable(MIPSOP_CTC2);
    MipsOp::enable(MIPSOP_MTC2);

    MipsOp::enable(MIPSOP_DIV);
    MipsOp::enable(MIPSOP_DIVU);

    /*********************** Component List ***********************/
    
    RTLComponent *addersub=new RTLComponent("addersub","slt");
    RTLComponent *logic_unit=new RTLComponent("logic_unit");
    RTLComponent *mul=new RTLComponent("mul","shift_stall");
    RTLComponent *div=new RTLComponent("div");

    RTLComponent *ifetch=new RTLComponent("ifetch","pipe_bpred_external");
    RTLComponent *data_mem=new RTLComponent("data_mem","external");
    RTLComponent *reg_file=new RTLComponent("reg_file","pipe");

    RTLComponent *pcadder=new RTLComponent("pcadder");
    RTLComponent *signext=new RTLComponent("signext16");
    RTLComponent *merge26lo=new RTLComponent("merge26lo");
    RTLComponent *branchresolve=new RTLComponent("branchresolve");
    RTLComponent *hi_reg=new RTLComponent("hi_reg");
    RTLComponent *lo_reg=new RTLComponent("lo_reg");

    RTLParameters const8_params;
    const8_params["VAL"]="0";
    RTLComponent *const8=new RTLComponent("const",const8_params);

    RTLParameters const16_params;
    const16_params["VAL"]="16";
    RTLComponent *const16=new RTLComponent("const",const16_params);

    RTLParameters const31_params;
    const31_params["VAL"]="31";
    RTLComponent *const31=new RTLComponent("const",const31_params);

    RTLProc proc("system");

    /*********************** Registers ***********************/

    RTLParameters width5_params;
    width5_params["WIDTH"]="5";
    RTLParameters width26_params;
    width26_params["WIDTH"]="26";
    RTLParameters width1_params;
    width1_params["WIDTH"]="1";

    RTLComponent *offset_reg1=new RTLComponent("pipereg");
    RTLComponent *instr_index_reg1=new RTLComponent("pipereg",width26_params);
    RTLComponent *sa_reg1=new RTLComponent("pipereg",width5_params);
    RTLComponent *dst_reg1=new RTLComponent("pipereg",width5_params);
    RTLComponent *pc_reg1=new RTLComponent("pipereg");

    RTLComponent *tgt_pc_reg1=new RTLComponent("pipereg");

    RTLComponent *result_reg2=new RTLComponent("pipereg");

    RTLComponent *jr_target_reg2=new RTLComponent("pipereg");
    RTLComponent *brresolve_reg2=new RTLComponent("pipereg",width1_params);

    RTLComponent *pc_fakereg1=new RTLComponent("fakedelay");

    /*********************** Other Components ***********************/

    RTLComponent *nop_rs=new RTLComponent("nop");
    RTLComponent *nop_rt=new RTLComponent("nop");
    RTLComponent *nop_result=new RTLComponent("nop");
    RTLComponent *nop_opB=new RTLComponent("nop");
    RTLComponent *rs_zeroer=new RTLComponent("zeroer",width5_params);
    RTLComponent *rt_zeroer=new RTLComponent("zeroer",width5_params);
    RTLComponent *rd_zeroer=new RTLComponent("zeroer",width5_params);

    /*********************** Stage Requests ***********************/
    proc.putInStage(const31,"op",1);
    proc.putInStage(const16,"op",2);
    proc.putInStage(const8,"op",2);
    proc.putInStage(hi_reg,"op",2);
    proc.putInStage(lo_reg,"op",2);
    proc.putInStage(ifetch,"pcreadop",1);
    proc.putInStage(addersub,"op",2);

    //mandatory
    proc.putInStage(result_reg2,"op",2);

    RTLComponent *cop2=new RTLComponent("cop2");
    proc.putInStage(cop2,"cfc2",2);
    proc.putInStage(cop2,"mctc2",2);
    /*********************** Datapath Wiring ***********************/

    proc.addConnection(ifetch,"rs",rs_zeroer,"d");
    proc.addConnection(ifetch,"rt",rt_zeroer,"d");
    proc.addConnection(rs_zeroer,"q",reg_file,"a_reg");
    proc.addConnection(rt_zeroer,"q",reg_file,"b_reg");

            // Stage 1 pipe registers
    proc.addConnection(ifetch,"offset",offset_reg1,"d");
    proc.addConnection(signext,"out",offset_reg1,"d");
    proc.addConnection(ifetch,"instr_index",instr_index_reg1,"d");
    proc.addConnection(ifetch,"sa",sa_reg1,"d");
    proc.addConnection(ifetch,"pc_out",pc_reg1,"d");

    proc.addConnection(ifetch,"rt",rd_zeroer,"d");
    proc.addConnection(ifetch,"rd",rd_zeroer,"d");
    proc.addConnection(const31,"out",rd_zeroer,"d");
    proc.addConnection(rd_zeroer,"q",dst_reg1,"d");

            // Conditional Branch path
    proc.addConnection(ifetch,"offset",signext,"in");
    proc.addConnection(signext,"out",pcadder,"offset");
    //proc.addConnection(offset_reg1,"q",pcadder,"offset");
    proc.addConnection(ifetch,"pc_out",pcadder,"pc");
    //proc.addConnection(pc_reg1,"q",pcadder,"pc");
    proc.addConnection(pcadder,"result",tgt_pc_reg1,"d");
    proc.addConnection(tgt_pc_reg1,"q",ifetch,"predict_tgt_pc");
    //proc.addConnection(pcadder,"result",ifetch,"load_data");

    proc.addConnection(branchresolve,"eq",brresolve_reg2,"d");
    proc.addConnection(branchresolve,"ne",brresolve_reg2,"d");
    proc.addConnection(branchresolve,"lez",brresolve_reg2,"d");
    proc.addConnection(branchresolve,"ltz",brresolve_reg2,"d");
    proc.addConnection(branchresolve,"gez",brresolve_reg2,"d");
    proc.addConnection(branchresolve,"gtz",brresolve_reg2,"d");
    proc.addConnection(brresolve_reg2,"q",ifetch,"load");

            // J and JAL path
    proc.addConnection(ifetch,"pc_out",merge26lo,"in1");
    //proc.addConnection(pc_reg1,"q",merge26lo,"in1");
    proc.addConnection(ifetch,"instr_index",merge26lo,"in2");
    //proc.addConnection(instr_index_reg1,"q",merge26lo,"in2");
    proc.addConnection(merge26lo,"out",tgt_pc_reg1,"d");
    //proc.addConnection(merge26lo,"out",ifetch,"load_data");

    proc.addConnection(ifetch,"pc_out",pc_fakereg1,"d");
    proc.addConnection(pc_fakereg1,"q",addersub,"opA");
    
            // JR and JALR path
    //proc.addConnection(nop_rs,"q",ifetch,"load_data");
    proc.addConnection(nop_rs,"q",jr_target_reg2,"d");
    proc.addConnection(jr_target_reg2,"q",ifetch,"load_data");

            // Other IR constant fanouts
    proc.addConnection(offset_reg1,"q",nop_opB,"d");

            // RS fanout
    proc.addConnection(reg_file,"a_readdataout",nop_rs,"d");

    proc.addConnection(nop_rs,"q",addersub,"opA");
    proc.addConnection(nop_rs,"q",logic_unit,"opA");
    proc.addConnection(nop_rs,"q",mul,"sa");
    proc.addConnection(nop_rs,"q",mul,"opA");
    proc.addConnection(nop_rs,"q",branchresolve,"rs");
    proc.addConnection(nop_rs,"q",div,"dividend");

            // RT fanout
    proc.addConnection(reg_file,"b_readdataout",nop_rt,"d");

    proc.addConnection(nop_rt,"q",nop_opB,"d");
    proc.addConnection(nop_rt,"q",mul,"opB");
    proc.addConnection(nop_rt,"q",data_mem,"d_writedata");
    proc.addConnection(nop_rt,"q",branchresolve,"rt");
    proc.addConnection(nop_rt,"q",div,"divider");

    proc.addConnection(nop_opB,"q",addersub,"opB");
    proc.addConnection(nop_opB,"q",logic_unit,"opB");
    proc.addConnection(nop_opB,"q",mul,"opA");

            // DIV
    proc.addConnection(div,"remainder",hi_reg,"d");
    proc.addConnection(div,"quotient",lo_reg,"d");

            // MUL
    proc.addConnection(mul,"hi",hi_reg,"d");
    proc.addConnection(mul,"lo",lo_reg,"d");
    proc.addControlConnection(dst_reg1,"q",mul,"dst");

            // Data memory
    proc.addConnection(addersub,"result",data_mem,"d_address");

            // Adder
    proc.addControlConnection(const8,"out",nop_opB,"d");

            // Shifter
    proc.addConnection(const16,"out",mul,"sa");
    proc.addConnection(sa_reg1,"q",mul,"sa");

        // Coprocessor 2 - Vector extension
    proc.addConnection(nop_rt,"q",cop2,"fromcpu");
    proc.addConnection(cop2,"tocpu",nop_result,"d");

            // Writeback
    proc.addConnection(addersub,"result",nop_result,"d");
    proc.addConnection(addersub,"result_slt",nop_result,"d");
    proc.addConnection(logic_unit,"result",nop_result,"d");
    proc.addConnection(mul,"shift_result",nop_result,"d");
    proc.addConnection(data_mem,"d_loadresult",nop_result,"d");
    proc.addConnection(hi_reg,"q",nop_result,"d");
    proc.addConnection(lo_reg,"q",nop_result,"d");

    proc.addConnection(nop_result,"q",reg_file,"c_writedatain");

            // Writeback destination
    proc.addConnection(dst_reg1,"q",reg_file,"c_reg");


  /*********************** Controller ***********************/
  PipelineOptions options;
  options.register_opcodes=false;

  // Setup Branch prediction
  options.bpred.predictor=true;
  options.bpred.predictorfile="bpred_1bittable.v";
  options.bpred.instructions.push_front(MIPSOP_BEQ);
  options.bpred.instructions.push_front(MIPSOP_BNE);
  options.bpred.instructions.push_front(MIPSOP_BLEZ);
  options.bpred.instructions.push_front(MIPSOP_BGTZ);
  options.bpred.instructions.push_front(MIPSOP_BLTZ);
  options.bpred.instructions.push_front(MIPSOP_BGEZ);
  options.bpred.instructions.push_front(MIPSOP_J);
  options.bpred.instructions.push_front(MIPSOP_JAL);
  options.bpred.predict_en_component=ifetch;
  options.bpred.predict_en_port="predict_en";
  options.bpred.predict_result_rdy_component=ifetch;
  options.bpred.predict_result_rdy_port="predict_result_rdy";
  options.bpred.predict_result_component=ifetch;
  options.bpred.predict_result_port="predict_result";

  CtrlPipelined control(&fdp,&proc,options);

  proc.addControlConnection(nop_result,"q",result_reg2,"d");
  control.insertPipeReg(result_reg2,1);

  /*********************** Hazard detection ***********************/
  HazardDetector *rs_haz=control.newHazardDetector(rs_zeroer,"q",dst_reg1,"q");
  HazardDetector *rt_haz=control.newHazardDetector(rt_zeroer,"q",dst_reg1,"q");
  //control.stallOnHazard(rs_haz,1);
  //control.stallOnHazard(rt_haz,1);
  control.bypassOnHazard(rs_haz,2,result_reg2,"q",nop_rs,"d");
  control.bypassOnHazard(rt_haz,2,result_reg2,"q",nop_rt,"d");

