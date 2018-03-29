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
        public MyHandler()
        {
        }

        public MyHandler(string[] args) : base(args)
        {
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
