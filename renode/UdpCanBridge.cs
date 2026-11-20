//
// UDP <-> CAN bridge for Renode, modelled on Renode's Linux-only SocketCANBridge.
//
// Every CAN frame seen on the Renode CANHub is sent to 127.0.0.1:<sendPort> as
// a 16-byte datagram, and datagrams arriving on 127.0.0.1:<listenPort> are
// put on the bus:
//
//   u32 id (little-endian) | u8 dlc | 3 bytes padding | 8 bytes data
//
// That's the same framing as port/host/hal_sim.c, so tools/harness can drive
// either the host simulator or the real firmware image in Renode unchanged.
//
// Load with `include @UdpCanBridge.cs`, then:
//   machine CreateUdpCanBridge "udpcan" <listenPort> <sendPort>
//   connector Connect sysbus.udpcan <canHub>
//
using System;
using System.Net;
using System.Net.Sockets;
using System.Threading;

using Antmicro.Renode.Core;
using Antmicro.Renode.Core.CAN;
using Antmicro.Renode.Core.Structure;
using Antmicro.Renode.Logging;

namespace Antmicro.Renode.Peripherals.CAN
{
    public static class UdpCanBridgeExtensions
    {
        public static void CreateUdpCanBridge(this IMachine machine, string name, int listenPort, int sendPort)
        {
            var bridge = new UdpCanBridge(listenPort, sendPort);
            machine.RegisterAsAChildOf(machine.SystemBus, bridge, NullRegistrationPoint.Instance);
            machine.SetLocalName(bridge, name);
        }
    }

    public class UdpCanBridge : ICAN
    {
        public UdpCanBridge(int listenPort, int sendPort)
        {
            socket = new UdpClient(new IPEndPoint(IPAddress.Loopback, listenPort));
            peer = new IPEndPoint(IPAddress.Loopback, sendPort);
            thread = new Thread(ReceiveLoop) { Name = "UdpCanBridge receive", IsBackground = true };
            thread.Start();
        }

        public void Reset()
        {
        }

        // A frame from the emulated bus: forward it to the host.
        public void OnFrameReceived(CANMessageFrame message)
        {
            if(message.ExtendedFormat || message.FDFormat || message.Data.Length > 8)
            {
                this.Log(LogLevel.Warning, "Dropping frame the UDP format can't carry: {0}", message);
                return;
            }
            var buf = new byte[FrameSize];
            BitConverter.GetBytes(message.Id).CopyTo(buf, 0); // host is little-endian
            buf[4] = (byte)message.Data.Length;
            Array.Copy(message.Data, 0, buf, 8, message.Data.Length);
            try
            {
                socket.Send(buf, buf.Length, peer);
            }
            catch(SocketException e)
            {
                this.Log(LogLevel.Debug, "UDP send failed: {0}", e.Message);
            }
        }

        public event Action<CANMessageFrame> FrameSent;

        // A datagram from the host: put it on the emulated bus.
        private void ReceiveLoop()
        {
            var from = new IPEndPoint(IPAddress.Any, 0);
            while(true)
            {
                byte[] buf;
                try
                {
                    buf = socket.Receive(ref from);
                }
                catch(ObjectDisposedException)
                {
                    return;
                }
                catch(SocketException)
                {
                    continue;
                }
                if(buf.Length != FrameSize || buf[4] > 8)
                {
                    this.Log(LogLevel.Warning, "Ignoring malformed datagram ({0} bytes)", buf.Length);
                    continue;
                }
                var id = BitConverter.ToUInt32(buf, 0) & 0x7FF;
                var data = new byte[buf[4]];
                Array.Copy(buf, 8, data, 0, data.Length);
                FrameSent?.Invoke(new CANMessageFrame(id, data));
            }
        }

        private const int FrameSize = 16;
        private readonly UdpClient socket;
        private readonly IPEndPoint peer;
        private readonly Thread thread;
    }
}
