using System.Collections;
using System.Collections.Generic;
using UnityEngine;


using Unity.Mathematics;
using Unity.MLAgents;

namespace MotorUpdate
{


#if USE_LSPD

    [CreateAssetMenu(fileName = "LSPD", menuName = "ScriptableObjects/LSPD", order = 1)]
    public class LSPDUpdateRule : MotorUpdateRule
    {


        [SerializeField]
        protected float dT = 1 / 60;




        private LSPDHierarchy _lpd;

        List<IArticulation> _motors;

        public override float GetTorque(float[] curState, float[] targetState)
        {

            return 0f;

        }

        public override float GetTorque(IState curState, IState targetState)
        {
            return 0f;
        }


        public override void Initialize(ModularMuscles muscles = null, float dT = 1 / 60  ) {

           

            this.dT = dT;



           _lpd = muscles.gameObject.AddComponent<LSPDHierarchy>();



            

            _motors = new List<IArticulation>();

           
            foreach(ArticulationBody ab in _lpd.Init(1000, GetActionTimeDelta(muscles.gameObject)) )
            {

                _motors.Add(new ArticulationBodyAdapter(ab));
                
            }




        }




    }


#endif

}