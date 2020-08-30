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
			SurviveContext api = new SurviveContext(args);
			while (api.Poll() == 0) {
			}
			api.Close();
		}
	}
}
