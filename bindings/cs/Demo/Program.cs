using libsurvive;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Demo
{
    
    class Program
    {
        static void Main(string[] args)
        {
            SurviveContext context = new SurviveContext(args);
            context.AddPoseUpdateCallback(HMDUpdate, -1);


            while (context.Poll() == 0) {
            }

        }

        public static void HMDUpdate(int ObjectID, Vector3 pos)
        {

        }

    }
}
