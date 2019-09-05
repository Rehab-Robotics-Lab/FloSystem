import React, { useEffect, useState } from 'react';
import * as ROSLIB from 'roslib';
import { sequencePropDef, sequenceContainerPropDef } from '../propTypes';

function Sequence({ sequence, setMovesList, getPoseSrv }) {
  return (
    <button
      type="button"
      onClick={() => {
        const movesListT = [];
        const seqLength = sequence.seq.arms.length;
        let numElements = 0;
        for (let idx = 0; idx < seqLength; idx += 1) {
          const poseId = sequence.seq.pose_ids[idx];
          const req = new ROSLIB.ServiceRequest({
            id: poseId,
          });
          getPoseSrv.callService(req, (res) => {
            movesListT[idx] = {
              lr: sequence.seq.arms[idx],
              pose: { id: poseId, pose: res.pose },
              status: 'not-run',
              time: sequence.seq.times[idx],
            };
            numElements += 1;
            if (numElements === seqLength) {
              setMovesList(movesListT);
            }
          });
        }
      }}
    >
      {sequence.seq.description}
    </button>
  );
}

Sequence.propTypes = sequencePropDef;

// Takes a parameter ros, which is the connection to ros
function SequenceContainer({
  ros, connected, setMovesList, MovesList,
}) {
  const [SeqList, setSeqList] = useState([]);
  const [showSave, setShowSave] = useState(false);
  const [saveID, setSaveID] = useState(0);
  const [saveDescription, setSaveDescription] = useState('');
  const [setSeqSrv, setSetSeqSrv] = useState(null);
  const [getPoseSrv, setGetPoseSrv] = useState(null);

  // get all of the poses
  useEffect(() => {
    if (!connected) return;

    const searchSeqClient = new ROSLIB.Service({
      ros,
      name: '/search_pose_seq',
      serviceType: 'flo_core/SearchPoseSeq',
    });

    const request = new ROSLIB.ServiceRequest({ search: '' });

    searchSeqClient.callService(request, (resp) => {
      const seqs = [];
      for (let i = 0; i < resp.ids.length; i += 1) {
        seqs.push({ id: resp.ids[i], seq: resp.sequences[i] });
      }
      setSeqList(seqs);
    });

    const setSeqSrvT = new ROSLIB.Service({
      ros,
      name: '/set_pose_seq',
      serviceType: 'flo_core/SetPoseSeq',
    });
    setSetSeqSrv(setSeqSrvT);

    const getPoseSrvT = new ROSLIB.Service({
      ros,
      name: '/get_pose_id',
      serviceType: 'flo_core/GetPoseID',
    });
    setGetPoseSrv(getPoseSrvT);
  }, [connected, ros]);


  return (
    <div
      id="sequences-container"
      style={{
        maxWidth: '150px', backgroundColor: 'white', borderRadius: '25px', padding: '10px', margin: '10px',
      }}
    >
      <h2>Sequences:</h2>


      <button type="button" onClick={() => { setShowSave(true); }} disabled={!connected}>Save Sequence</button>
      <hr />
      {showSave
        && (
        <div style={{
          position: 'absolute', bottom: 0, left: 0, right: 0, top: 0, background: 'rgba(0,0,0,.3)', display: 'flex', justifyContent: 'center', alignItems: 'center',
        }}
        >
          <div style={{
            width: '200px', background: 'white', borderRadius: '10px', minWidth: '500px', position: 'relative', textAlign: 'center', display: 'flex', flexDirection: 'column',
          }}
          >
            <h3>Save a New Sequence or Overwrite an Existing One</h3>
            <label htmlFor="saveSeqIDSelector">
Save As:
              <select
                id="saveSeqIDSelector"
                onChange={(obj) => {
                  const newId = parseInt(obj.target.value, 10);
                  setSaveID(newId);
                  if (newId > 0) {
                    const newDesc = obj.target[obj.target.selectedIndex].textContent;
                    setSaveDescription(newDesc);
                  }
                }}
              >
                <option value="0">New Sequence</option>
                {
                SeqList.map((value) => (
                  <option value={value.id}>{value.seq.description}</option>
                ))
                }

              </select>
            </label>
            <label htmlFor="saveSeqDescription">
                Description:
              <input type="text" value={saveDescription} onChange={(obj) => { setSaveDescription(obj.target.value); }} />
            </label>

            <button
              type="button"
              disabled={
                    !connected || !saveDescription
}
              onClick={() => {
                const poseIds = [];
                const times = [];
                const arms = [];
                let totalTime = 0;
                for (let idx = 0; idx < MovesList.length; idx += 1) {
                  poseIds.push(MovesList[idx].pose.id);
                  times.push(MovesList[idx].time);
                  arms.push(MovesList[idx].lr);
                  totalTime += MovesList[idx].time;
                }

                const newSeq = {
                  pose_ids: poseIds,
                  times,
                  arms,
                  description: saveDescription,
                  total_time: totalTime,
                };
                const req = new ROSLIB.ServiceRequest({
                  sequence: newSeq,
                  id: saveID,
                });

                setSeqSrv.callService(req, (res) => {
                  const targetId = SeqList.findIndex((item) => (item.id === res.id));
                  const SeqListT = [...SeqList];
                  if (targetId === -1) {
                    SeqListT.push({
                      id: res.id,
                      seq: newSeq,
                    });
                  } else { SeqListT[targetId] = { id: res.id, seq: newSeq }; }
                  setSeqList(SeqListT);
                  setShowSave(false);
                  setSaveID(0);
                  setSaveDescription('');
                });


                // ros serve save id
              }}
            >
Save
            </button>
            <button type="button" onClick={() => { setShowSave(false); }}>Cancel</button>
          </div>
        </div>
        )}


      <div style={{
        display: 'flex', flexDirection: 'column', overflow: 'auto', maxHeight: '400px',
      }}
      >
        {
                SeqList.map((value) => (
                  <Sequence
                    id={value.id}
                    sequence={value}
                    setMovesList={setMovesList}
                    getPoseSrv={getPoseSrv}
                  />
                ))
            }
      </div>
    </div>
  );
}

SequenceContainer.defaultProps = {
  ros: null,
};

SequenceContainer.propTypes = sequenceContainerPropDef;

export default SequenceContainer;
