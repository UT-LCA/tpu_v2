#Please Run This Script Before Compiling
if {![info exists ::ppl_instance_name]} {set ::ppl_instance_name {}} 
set dq_pin_name ${::ppl_instance_name}mem_dq
set dqs_pin_name ${::ppl_instance_name}mem_dqs
set dm_pin_name ${::ppl_instance_name}mem_dm
set mem_clk_pin_name ${::ppl_instance_name}mem_clk
set mem_clk_n_pin_name ${::ppl_instance_name}mem_clk_n
set dqsn_pin_name ${::ppl_instance_name}mem_dqsn
set_instance_assignment -name DQ_GROUP 9 -to ${dq_pin_name}\[0..7\] -from ${dqs_pin_name}\[0\]
set_instance_assignment -name DQ_GROUP 9 -to ${dm_pin_name}\[0\] -from ${dqs_pin_name}\[0\]
set_instance_assignment -name DQSB_DQS_PAIR ON -from ${dqsn_pin_name}\[0\] -to ${dqs_pin_name}\[0\]
set_instance_assignment -name DQ_GROUP 9 -to ${dq_pin_name}\[8..15\] -from ${dqs_pin_name}\[1\]
set_instance_assignment -name DQ_GROUP 9 -to ${dm_pin_name}\[1\] -from ${dqs_pin_name}\[1\]
set_instance_assignment -name DQSB_DQS_PAIR ON -from ${dqsn_pin_name}\[1\] -to ${dqs_pin_name}\[1\]
set_instance_assignment -name DQ_GROUP 9 -to ${dq_pin_name}\[16..23\] -from ${dqs_pin_name}\[2\]
set_instance_assignment -name DQ_GROUP 9 -to ${dm_pin_name}\[2\] -from ${dqs_pin_name}\[2\]
set_instance_assignment -name DQSB_DQS_PAIR ON -from ${dqsn_pin_name}\[2\] -to ${dqs_pin_name}\[2\]
set_instance_assignment -name DQ_GROUP 9 -to ${dq_pin_name}\[24..31\] -from ${dqs_pin_name}\[3\]
set_instance_assignment -name DQ_GROUP 9 -to ${dm_pin_name}\[3\] -from ${dqs_pin_name}\[3\]
set_instance_assignment -name DQSB_DQS_PAIR ON -from ${dqsn_pin_name}\[3\] -to ${dqs_pin_name}\[3\]
set_instance_assignment -name DQ_GROUP 9 -to ${dq_pin_name}\[32..39\] -from ${dqs_pin_name}\[4\]
set_instance_assignment -name DQ_GROUP 9 -to ${dm_pin_name}\[4\] -from ${dqs_pin_name}\[4\]
set_instance_assignment -name DQSB_DQS_PAIR ON -from ${dqsn_pin_name}\[4\] -to ${dqs_pin_name}\[4\]
set_instance_assignment -name DQ_GROUP 9 -to ${dq_pin_name}\[40..47\] -from ${dqs_pin_name}\[5\]
set_instance_assignment -name DQ_GROUP 9 -to ${dm_pin_name}\[5\] -from ${dqs_pin_name}\[5\]
set_instance_assignment -name DQSB_DQS_PAIR ON -from ${dqsn_pin_name}\[5\] -to ${dqs_pin_name}\[5\]
set_instance_assignment -name DQ_GROUP 9 -to ${dq_pin_name}\[48..55\] -from ${dqs_pin_name}\[6\]
set_instance_assignment -name DQ_GROUP 9 -to ${dm_pin_name}\[6\] -from ${dqs_pin_name}\[6\]
set_instance_assignment -name DQSB_DQS_PAIR ON -from ${dqsn_pin_name}\[6\] -to ${dqs_pin_name}\[6\]
set_instance_assignment -name DQ_GROUP 9 -to ${dq_pin_name}\[56..63\] -from ${dqs_pin_name}\[7\]
set_instance_assignment -name DQ_GROUP 9 -to ${dm_pin_name}\[7\] -from ${dqs_pin_name}\[7\]
set_instance_assignment -name DQSB_DQS_PAIR ON -from ${dqsn_pin_name}\[7\] -to ${dqs_pin_name}\[7\]
