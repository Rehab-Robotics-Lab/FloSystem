/*
  BOLIDE_Player.cpp - Modified for XYZrobot ATmega 1280 control board.
  Copyright (c) 2015 Wei-Shun You. XYZprinting Inc.  All right reserved.
*/
/*
  BioloidController.cpp - ArbotiX Library for Bioloid Pose Engine
  Copyright (c) 2008-2012 Michael E. Ferguson.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "BOLIDE_Player.h"

static char packet_send[110];
static unsigned int checksum_1;
static unsigned int checksum_2;

/* new-style setup */
void BOLIDE_Player::setup(long baud, int servo_cnt){
    static int i;
	A1_16_Ini(baud);
    // setup storage
    id_ = (unsigned char *) malloc(servo_cnt * sizeof(unsigned char));
    pose_ = (unsigned int *) malloc(servo_cnt * sizeof(unsigned int));
    nextpose_ = (unsigned int *) malloc(servo_cnt * sizeof(unsigned int));
    speed_ = (int *) malloc(servo_cnt * sizeof(int));
    // initialize
    poseSize = servo_cnt;
    for(i=0;i<poseSize;i++){
        id_[i] = i+1;
        pose_[i] = 512<<A1_16_SHIFT;
        nextpose_[i] = 512<<A1_16_SHIFT;
    }
    interpolating = 0;
    playing = 0;
    lastframe_ = millis();
}
void BOLIDE_Player::setId(int index, int id){
    id_[index] = id;
}
int BOLIDE_Player::getId(int index){
    return id_[index];
}

/* load a named pose from FLASH into nextpose. */
void BOLIDE_Player::loadPose( const unsigned int * addr ){
    int i;
    poseSize = pgm_read_word_near(addr); // number of servos in this pose
    for(i=0; i<poseSize; i++)
        nextpose_[i] = pgm_read_word_near(addr+1+i) << A1_16_SHIFT;
}
/* read in current servo positions to the pose. */
void BOLIDE_Player::readPose(){
    for(int i=0;i<poseSize;i++){
        pose_[i] = ReadPosition(id_[i])<<A1_16_SHIFT;
		delay(25);   
    }
}
/* write pose out to servos using sync write. */
void BOLIDE_Player::writePose(){
	static int temp;
	static char _i, _j;
	packet_send[0] = 0xff;
	packet_send[1] = 0xff;
	packet_send[2] = 7+5*poseSize;
	packet_send[3] = 0xfe;
	packet_send[4] = CMD_I_JOG;
	checksum_1 = packet_send[2]^packet_send[3]^packet_send[4];
	for(_i = 0;_i < poseSize;_i++){
		temp = pose_[_i] >> A1_16_SHIFT;
        packet_send[7+5*_i] = temp&0xff;
        checksum_1 ^= packet_send[7+5*_i];
        packet_send[8+5*_i] = (temp&0xff00)>>8;
        checksum_1 ^= packet_send[8+5*_i];
        packet_send[9+5*_i] = 0;
        checksum_1 ^= packet_send[9+5*_i];
        packet_send[10+5*_i] = _i+1;
        checksum_1 ^= packet_send[10+5*_i];
        packet_send[11+5*_i] = 2;
        checksum_1 ^= packet_send[11+5*_i];
    }
	checksum_1 &= 0xfe;
	packet_send[5] = checksum_1;
	checksum_2 = (~checksum_1)&0xfe;
	packet_send[6] = checksum_2;
	for(_j = 0;_j<packet_send[2];_j++) Serial1.write(packet_send[_j]);
}

/* set up for an interpolation from pose to nextpose over TIME 
    milliseconds by setting servo speeds. */
void BOLIDE_Player::interpolateSetup(int time){
    int i;
    int frames = (time/A1_16_FRAME_LENGTH) + 1;
	total_frame = frames;				//Wei-Shun You edits: record the frames between poses
    lastframe_ = millis();
    // set speed each servo...
    for(i=0;i<poseSize;i++){
        if(nextpose_[i] > pose_[i]){
            speed_[i] = (nextpose_[i] - pose_[i])/frames + 1;
        }else{
            speed_[i] = (pose_[i]-nextpose_[i])/frames + 1;
        }
    }
    interpolating = 1;
}
/* interpolate our pose, this should be called at about 30Hz. */
void BOLIDE_Player::interpolateStep(){
    if(interpolating == 0) return;
    int i;
    int complete = poseSize;
    while(millis() - lastframe_ < A1_16_FRAME_LENGTH);
	frame_counter++;
    lastframe_ = millis();
    // update each servo
    for(i=0;i<poseSize;i++){
        int diff = nextpose_[i] - pose_[i];
        if(diff == 0){
            complete--;
        }else{
            if(diff > 0){
                if(diff < speed_[i]){
                    pose_[i] = nextpose_[i];
                    complete--;
                }else
                    pose_[i] += speed_[i];
            }else{
                if((-diff) < speed_[i]){
                    pose_[i] = nextpose_[i];
                    complete--;
                }else
                    pose_[i] -= speed_[i];                
            }       
        }
    }
    if((complete <= 0) && (frame_counter >= total_frame)) {
		interpolating = 0;
		frame_counter = 0;
	}
    writePose();      
}

/* get a servo value in the current pose */
int BOLIDE_Player::getCurPose(int id){
    for(int i=0; i<poseSize; i++){
        if( id_[i] == id )
            return ((pose_[i]) >> A1_16_SHIFT);
    }
    return -1;
}
/* get a servo value in the next pose */
int BOLIDE_Player::getNextPose(int id){
    for(int i=0; i<poseSize; i++){
        if( id_[i] == id )
            return ((nextpose_[i]) >> A1_16_SHIFT);
    }
    return -1;
}
/* set a servo value in the next pose */
void BOLIDE_Player::setNextPose(int id, int pos){
    for(int i=0; i<poseSize; i++){
        if( id_[i] == id ){
            nextpose_[i] = (pos << A1_16_SHIFT);
            return;
        }
    }
}

/* play a sequence. */
void BOLIDE_Player::playSeq( const transition_t  * addr ){
    sequence = (transition_t *) addr;
    // number of transitions left to load
    transitions = pgm_read_word_near(&sequence->time);
    sequence++;    
    // load a transition
    loadPose((const unsigned int *)pgm_read_word_near(&sequence->pose));
    interpolateSetup(pgm_read_word_near(&sequence->time));
    transitions--;
    playing = 1;
}
/* keep playing our sequence */
void BOLIDE_Player::play(){
    if(playing == 0) return;
    if(interpolating > 0){
        interpolateStep();
    }else{  // move onto next pose
        sequence++;   
        if(transitions > 0){
            loadPose((const unsigned int *)pgm_read_word_near(&sequence->pose));
            interpolateSetup(pgm_read_word_near(&sequence->time));
            transitions--;
        }else{
            playing = 0;
        }
    }
}

