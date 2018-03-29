using libsurvive;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Demo
{
    internal class MyHandler : SurviveContext
    {
        private static void WritePose(string name, SurvivePose pose)
        {
            Console.Out.WriteLine(name);
            Console.Out.Write(" [ ");
            for (int i = 0; i < 3; i++)
                Console.Out.Write("{0} ", pose.Pos[i]);
            Console.Out.Write(" ] [ ");
            for (int i = 0; i < 4; i++)
                Console.Out.Write("{0} ", pose.Rot[i]);
            Console.Out.Write(" ] ");
            Console.Out.WriteLine();
        }

        public MyHandler() : base()
        {
        }

        public MyHandler(string[] args) : base(args)
        {
        }

        protected void LightHouseEvent1(IntPtr ctx, byte lighthouse, SurvivePose lighthouse_pose, SurvivePose object_pose)
        {
            base.LightHouseEvent(ctx, lighthouse, lighthouse_pose, object_pose);
            WritePose("Lighthouse", lighthouse_pose);
            WritePose("Object", object_pose);
        }
     
        protected override void PoseEvent(IntPtr so, byte lighthouse, SurvivePose pose)
        {
            WritePose("Pose", pose);
            base.PoseEvent(so, lighthouse, pose);
        }
    }
    class Program
    {
        static void Main(string[] args)
        {
            MyHandler handler = new MyHandler(args);
            
            while (handler.Poll() == 0) {
            }

        }

    }
}
