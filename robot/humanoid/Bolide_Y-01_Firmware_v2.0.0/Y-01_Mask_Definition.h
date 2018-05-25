#ifndef MASK_DEFINITION_H
#define MASK_DEFINITION_H

#define max_pose_index             100
#define max_seq_index              250

#define packet_header              0xFF
#define packet_tail                0xFE

#define header_address             0x00
#define length_address             0x01
#define CMD_address                0x02
#define motor_ID_address           0x03

#define motor_num_address          0x03
#define motor_pos_msb              0x04
#define motor_pos_lsb              0x05

#define CMD_version_read	       0x00
#define CMD_init_motor             0x01
#define CMD_set_motor              0x02
#define CMD_capture_motor          0x03
#define CMD_relax_motor            0x04
#define CMD_SN_read		           0x05
#define CMD_capture_current        0x06
#define CMD_capture_temp           0x07

#define CMD_SEQ_load_PoseCnt       0x10
#define CMD_SEQ_load_Pose          0x11
#define CMD_SEQ_load_SEQCnt        0x12
#define CMD_SEQ_load_SEQ           0x13
#define CMD_SEQ_loop_load_PoseCnt  0x14
#define CMD_SEQ_relax              0x20
#define CMD_SEQ_halt               0x30

#define SEQ_Process_load_PoseCnt   0x01
#define SEQ_Process_load_Pose      0x02
#define SEQ_Process_load_SEQCnt    0x03
#define SEQ_Process_load_SEQ       0x04

#define seq_pose_cnt_address       0x03
#define seq_pose_ID_address        0x03
#define seq_seq_cnt_address        0x03
#define seq_pose_start_address     0x04
#define seq_pose_name_address      0x03
#define seq_pose_time_MSB_address  0x04
#define seq_pose_time_LSB_address  0x05

#define CMD_packet_error           0xFD
#define packet_timeout             0x01
#define packet_error               0x02

#define model_Bolide		   0x42
#define type_Y01		   0x01
#define type_Crawler		   0x02
#define type_Others		   0xFF
#define application_default	   0x44
#define main_version_number	   0x01
#define secondary_version_number   0x02
#define revision_number		   0x08

#endif

