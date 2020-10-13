using libsurvive;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Demo
{
    
    class Program
    {
		static void Main() {
			string[] args = System.Environment.GetCommandLineArgs();
			var api = new SurviveAPI(args);

			while (api.WaitForUpdate()) {
				SurviveAPIOObject obj;
				while ((obj = api.GetNextUpdated()) != null) {
					Console.WriteLine(obj.Name + "(" + obj.SerialNumber + ") : " + obj.LatestPose);
				}
			}

			api.Close();
		}
	}
}
