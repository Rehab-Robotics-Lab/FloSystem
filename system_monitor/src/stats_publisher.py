#!/usr/bin/env python
"""A mdoule which allows monitoring and publishing system stats"""

from __future__ import division
import rospy
import psutil
import re
import subprocess

from system_monitor.msg import CPUutil, HDDutil, MEMutil, NETstats


class StatsPublisher(object):

    def __init__(self):
        rospy.init_node('stats_publisher')

        self.cpu_stats_pub = rospy.Publisher(
            'cpu_stats', CPUutil, queue_size=20)
        self.mem_stats_pub = rospy.Publisher(
            'mem_stats', MEMutil, queue_size=20)
        self.net_stats_pub = rospy.Publisher(
            'net_stats', NETstats, queue_size=20)
        self.hdd_stats_pub = rospy.Publisher(
            'hdd_stats', HDDutil, queue_size=20)

        rospy.Timer(rospy.Duration(1), self.read_cpu)
        rospy.Timer(rospy.Duration(20), self.read_disk)
        rospy.Timer(rospy.Duration(3), self.read_mem)
        rospy.Timer(rospy.Duration(2), self.read_net)
        rospy.spin()

    def read_cpu(self, _):
        """Read the current CPU level"""
        cpu_load = psutil.cpu_percent(interval=None, percpu=False)
        rospy.logdebug('cpu load: %5.2f%%', cpu_load)
        msg = CPUutil()
        msg.percent_utilization = cpu_load
        self.cpu_stats_pub.publish(msg)

    def read_disk(self, _):
        """Read the current disk utlization"""
        disk_stats = psutil.disk_usage('/')
        disk_percent_free = 100-disk_stats.percent
        rospy.logdebug('disk has %5.2f%% free', disk_percent_free)
        msg = HDDutil()
        msg.percent_free = disk_percent_free
        self.hdd_stats_pub.publish(msg)

    def read_mem(self, _):
        """Read the current memory utilization"""
        mem_stats = psutil.virtual_memory()
        mem_load = 100 * (mem_stats.total -
                          mem_stats.available)/mem_stats.total
        rospy.logdebug('mem percent used: %5.2f%%', mem_load)
        msg = MEMutil()
        msg.percent_used = mem_load
        self.mem_stats_pub.publish(msg)

    def read_net(self, _):
        """Read the current network status"""
        net_stats = self.get_net_strength()
        if net_stats:
            link_quality = net_stats['link_quality']
            signal_strength = net_stats['signal_level']
            rospy.logdebug('Network Signal Strength: %5.2fdb Link Quality: %4.1f%%',
                           signal_strength, link_quality)
            msg = NETstats()
            msg.link_quality = link_quality
            msg.signal_strength = signal_strength
            self.net_stats_pub.publish(msg)
        else:
            rospy.logdebug('could not get network stats')

    @staticmethod
    def get_net_strength():
        """Get the network strength

        Return:
            A dictionary with the fields `link_quality` and `signal_level`
        """
        try:
            proc = subprocess.Popen('/sbin/iwconfig | grep Link', shell=True,
                                    stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            output, err = proc.communicate()
            if not output:
                if err:
                    rospy.logdebug(
                        'no output received while checking net strength, got error:\n%s', err)
                else:
                    rospy.logdebug(
                        'no output received while checking net strength')

            msg = output.decode('utf-8')
            lqv = re.split(
                '/', re.search('(?<=Link Quality=)[0-9/]*', msg).group(0))
            link_quality = 100*int(lqv[0])/int(lqv[1])
            signal_level = int(
                re.search('(?<=Signal level=)[0-9\-]*', msg).group(0))
            return {'link_quality': link_quality, 'signal_level': signal_level}
        except:
            return None


if __name__ == "__main__":
    StatsPublisher()
