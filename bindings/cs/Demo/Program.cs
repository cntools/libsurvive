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
            LibSurViveAPI api = LibSurViveAPI.Instance;

            var so = api.GetSurviveObjectByName("HMD");
            
        }

        public static void HMDUpdate(int ObjectID, Vector3 pos)
        {

        }

    }
}
