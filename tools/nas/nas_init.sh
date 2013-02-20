#!/bin/bash

echo "version 2.5"

# LOG:
# 2.5:
#   1. XFS mount options updated.
#   2. RAID1 chunk size updated.
#   3. splice enabled for XFS.
# 2.4:
#   1. support RAID-1
#   2. splice enabled by default 
# 2.3:
#   1. fixing the mkdir share for loop
# 2.2:
#   1. automatic size settings for PARTSIZE
# 2.1:
#   1. setting coal to 100 in net queue 0 and fixing for in NETQ


#
# t= topology [sd|rd0|rd1|rd5]
# p= prepare drives (fdisk)
# m= mkfs+mdadm
# b= burn new rootfs to /dev/sda1
# f= file system format (ext4/xfs/btrfs)
# s= for SSD medium
# l= for largepage support (64k)
# h= for HDDs number (4/8) in RAID5 configuration

PREPARE_HDD="no"
ZAP_ROOTFS="no"
MKFS="no"
TOPOLOGY="rd5"
HDD_NUM="4"
ARCH=`uname -m`
FS="ext4"
NFS="no"
SSD="no"
CPU_COUNT=`grep -c ^processor /proc/cpuinfo`
COPY_CMD="/usr/sbin/bar -n"
ROOTFS_DEVICE="/dev/nfs"
LARGE_PAGE="no"
PARTNUM="1"
PARTSIZE="55GB"
NETQ="0"

ARGV0=$(basename ${0} | cut -f1 -d'.')
STAMP="[${ARGV0}]/[$(date +%H:%M-%d%b%Y)]:"

function do_error {
	echo -e "\n${STAMP}Error: ${1}"
	exit 1
}

if [ "$CPU_COUNT" == "0" ]; then
	CPU_COUNT="1"
fi

# verify supporting arch
case "$ARCH" in
	armv5tel | armv6l | armv7l)	echo "Architecture: ${ARCH}" ;;
	*)				do_error "Architecture ${ARCH} unsupported" ;;
esac

while getopts "lpbmn:sf:t:h:" flag; do
	case "$flag" in
		f)
			FS=$OPTARG
			case "$OPTARG" in
				ext4|btrfs|xfs|NONE)	echo "Filesystem: ${OPTARG}" ;;
				*)			do_error "Usage: file-system: ext4|xfs|btrfs" ;;
			esac
		;;

		s)	SSD="yes"
		;;

		b)	ZAP_ROOTFS="yes"
			ROOTFS_TARBALL="$OPTARG"
		;;

		m)	MKFS="yes"
		;;

		n)	PARTNUM="$OPTARG"
		;;

		p)	PREPARE_HDD="yes"
		;;

		l)	LARGE_PAGE="yes"
		;;

		t)	TOPOLOGY=$OPTARG
			case "$OPTARG" in
				sd|rd0|rd1|rd5) ;;
				*)	do_error "Usage: drive toplogy: sd|rd0|rd1|rd5" ;;
			esac
		;;

		h)	HDD_NUM=$OPTARG
			case "$OPTARG" in
				4|8) ;;
				*)	do_error "Usage: drive toplogy: 4|8" ;;
			esac
		;;

		*)	echo "Usage: $0"
			echo "           -f <ext4|xfs|btrfs|fat32>: file system type ext4, xfs, btrfs or fat32"
			echo "           -t <sd|rd0|rd1|rd5>: drive topology"
			echo "           -n <num>: partition number to be mounted"
			echo "           -m: mkfs/mdadm"
			echo "           -p: prepare drives"
			echo "           -b <rootfs_tarball_path>:  path to rootfs tarball to be placed on /dev/sda2"
			exit 1
		;;
	esac
	echo "$flag" $OPTIND $OPTARG
done

echo -ne "system:\n"
case "$TOPOLOGY" in
	sd)	echo -ne " Topology    Single drive\n" ;;
	rd0)	echo -ne " Topology    RAID0\n" ;;
	rd1)	echo -ne " Topology    RAID1\n" ;;
	rd5)	echo -ne " Topology    RAID5\n" ;;
	*)	do_error " Topolgy     Invalid" ;;
esac

echo -ne " Filesystem  $FS\n"
echo -ne " NFS         $NFS\n"
echo -ne " MKFS        $MKFS\n"
echo -ne " SSD         $SSD\n"
echo -ne " ARCH        $ARCH\n"
echo -ne " Cores       $CPU_COUNT\n"
echo -ne " Large Page  $LARGE_PAGE\n\n"

echo "going to:"
[ "$PREPARE_HDD" == "yes" ]	&&	echo "- format HDDs"
[ "$MKFS" == "yes" ]		&&	echo "- perform mkfs/mdadm"
[ "$ZAP_ROOTFS" == "yes" ]	&&	echo "- burn rootfs ${ROOTFS_TARBALL} to ${ROOTFS_DEVICE}"

#read -p "do you want to continue?:" ans_yn
#case "$ans_yn" in
#	[Yy]) echo "OK ..." ;;
#	*) exit 1 ;;
#esac

if [ "$MKFS" == "yes" ]; then
	case "$FS" in
		xfs)	[ ! -e "$(which mkfs.xfs)" ]	&&	do_error "missing mkfs.xfs in rootfs (aptitude install xfsprogs)" ;;
		ext4)	[ ! -e "$(which mkfs.ext4)" ]	&&	do_error "missing mkfs.ext4 in rootfs (aptitude install e2fsprogs)" ;;
		fat32)	[ ! -e "$(which mkdosfs)" ]	&&	do_error "missing mkdosfs in rootfs (aptitude install dosfstools)" ;;
		btrfs)	[ ! -e "$(which mkfs.btrfs)" ]	&&	do_error "missing mkfs.btrfs in rootfs (aptitude install btrfs-tools)" ;;
		NONE)						echo "no filesystem specified" ;;
		*)						do_error "no valid filesystem specified" ;;
	esac
fi

[[ "$TOPOLOGY" == "rd0" || "$TOPOLOGY" == "rd1" || "$TOPOLOGY" == "rd5" ]] && [ ! -e "$(which mdadm)" ] && do_error "missing mdadm in rootfs (aptitude install mdadm)"

#if [[ -e "$(which smbd)" && -e "$(which nmbd)" ]]; then
#    echo -n "* Starting Samba daemons"
#    $(which smbd) && $(which nmbd)
#else

if [ "$(pidof smbd)" ]; then
	echo -ne "killall smbd\n"
	killall smbd
fi

if [ "$(pidof nmbd)" ]; then
	echo -ne "killall nmbd\n"
	killall nmbd
fi

if [ `mount | grep mnt | grep -v grep | wc -l` != 0 ]; then
	echo -ne "umount /mnt\n"
	umount /mnt
fi

#[ -e /dev/md_d0 ] && mdadm --manage --stop /dev/md_d0
#[ -e /dev/md_d1 ] && mdadm --manage --stop /dev/md_d1

if [ "$TOPOLOGY" == "sd" ]; then
	DRIVES=a
	PARTSIZE="55GB"
elif [ "$TOPOLOGY" == "rd0" ]; then
	DRIVES="b c"
	PARTSIZE="30GB"
elif [ "$TOPOLOGY" == "rd1" ]; then
	DRIVES="a b"
	PARTSIZE="55GB"
elif [ "$TOPOLOGY" == "rd5" ]; then
    if [ "$HDD_NUM" == "8" ]; then
	DRIVES="a b c d e f g h"
	PARTSIZE="10GB"
    elif [ "$HDD_NUM" == "4" ]; then
	DRIVES="a b c d"
	PARTSIZE="20GB"
    fi
fi

if [ "$PREPARE_HDD" == "yes" ]; then
	[ ! -e "$(which fdisk)" ] && do_error "missing fdisk in rootfs"

	set -o verbose
	if [ "$SSD" == "no" ]; then
		for drive in `echo $DRIVES`; do echo -e "c\no\nn\np\n1\n4096\n+${PARTSIZE}\nt\n83\nw\n" | fdisk -u /dev/sd${drive}; done
	else
		for drive in `echo $DRIVES`; do echo -e "c\no\nn\np\n1\n63\n+${PARTSIZE}\nt\n83\nw\n" | fdisk  -H 224 -S 56 /dev/sd${drive}; done
#		for drive in `echo $DRIVES`; do echo -e "c\no\nn\np\n1\n2\n+${PARTSIZE}\nt\n83\nw\n" | fdisk -H 32 -S 32 /dev/sd${drive}; done
	fi
	set +o verbose
	fdisk -ul

	blockdev --rereadpt /dev/sd${drive} || do_error "============= The partition table has been altered, please reboot device =============="
fi

if [ "$ZAP_ROOTFS" == "yes" ]; then
	[ ! -e "$ROOTFS_TARBALL" ] && do_error "missing rootfs tarball, use option -n <rootfs_tarball_path>"

	mkfs.ext3 ${ROOTFS_DEVICE}
	mount ${ROOTFS_DEVICE} /mnt
	cd /mnt
	$COPY_CMD "$ROOTFS_TARBALL" | tar xpjf -

	# overwrite tarball files with NFS based files
	#cp /etc/hostname           /mnt/etc/hostname
	#cp /usr/sbin/nas_init.sh   /mnt/usr/sbin/
	#cp /etc/network/interfaces /mnt/etc/network/interfaces
	cd /root
	umount /mnt
	sleep 3
fi

echo -ne "\n==================================================\n"
echo -ne "\nnetwork optimisation\n"

if [ "$ARCH" == "armv5tel" ]; then
# KW section, LSP_5.1.3
	set -o verbose
	mv_eth_tool -txdone 8
	mv_eth_tool -txen 0 1
	mv_eth_tool -reuse 1
	mv_eth_tool -recycle 1
	mv_eth_tool -lro 0 1
	mv_eth_tool -rxcoal 0 160
	set +o verbose
elif [[ "$ARCH" == "armv6l" || "$ARCH" == "armv7l" ]]; then
# AXP / KW40
	set -o verbose
	echo 1 > /sys/devices/platform/neta/gbe/skb
	ethtool  --offload eth0 gso on
	ethtool  --offload eth1 gso on
	ethtool  --offload eth2	gso on
	ethtool  --offload eth3 gso on
	ethtool  --offload eth0 tso on
	ethtool  --offload eth1 tso on
	ethtool  --offload eth2 tso on
	ethtool  --offload eth3 tso on

	# following offload lower NAS BM, we do not use it
	#ethtool  --offload eth0 gro on
	#ethtool  --offload eth1 gro on
	#ethtool  --offload eth0 tso on
	#ethtool  --offload eth1 tso on

	# set RX coalecing to zero on all port 0 queues
	for (( i=0; i<=$NETQ; i++ )); do
		echo "0 $i 100" > /sys/devices/platform/neta/gbe/rxq_time_coal
		echo "1 $i 100" > /sys/devices/platform/neta/gbe/rxq_time_coal
		//echo "3 $i 100" > /sys/devices/platform/neta/gbe/rxq_time_coal
		//echo "3 $i 100" > /sys/devices/platform/neta/gbe/rxq_time_coal
	done
	ethtool -k eth0
	ethtool -k eth1
	ethtool -k eth2
	ethtool -k eth3

	set +o verbose
else
	do_error "Unsupported ARCH=$ARCH"
fi

echo -ne "\n==================================================\n"
sleep 1
echo -ne "\n==================================================\n"

if [ "$TOPOLOGY" == "rd5" ]; then
	echo -ne "\nStarting RAID5 build\n"
	for drive in `echo $DRIVES`; do PARTITIONS="${PARTITIONS} /dev/sd${drive}${PARTNUM}"; done

	set -o verbose

	for drive in `echo $DRIVES`; do echo -e 1024 > /sys/block/sd${drive}/queue/read_ahead_kb; done

	if [ "$MKFS" == "yes" ]; then
		[ -e /dev/md0 ] && mdadm --manage --stop /dev/md0

		for partition in `echo $DRIVES`; do mdadm --zero-superblock /dev/sd${partition}1; done

		if [ "$SSD" == "no" ]; then
			echo "y" | mdadm --create -c 128 /dev/md0 --level=5 -n $HDD_NUM --force $PARTITIONS
		else
			# most SSD use eraseblock of 512, so for performance reasons we use it
			echo "y" | mdadm --create -c 512 /dev/md0 --level=5 -n $HDD_NUM --force $PARTITIONS
		fi

		if [ "$FS" == "ext4" ]; then
			if [ "$SSD" == "no" ]; then
			    if [ "$HDD_NUM" == "8" ]; then
				mkfs.ext4 -j -m 0 -T largefile -b 4096 -E stride=32,stripe-width=224 -F /dev/md0
			    elif [ "$HDD_NUM" == "4" ]; then
				mkfs.ext4 -j -m 0 -T largefile -b 4096 -E stride=32,stripe-width=96 -F /dev/md0
			    fi
			else
				mkfs.ext4 -j -m 0 -T largefile -b 4096 -E stride=128,stripe-width=384 -F /dev/md0
			fi
		elif [ "$FS" == "xfs" ]; then
			mkfs.xfs -f /dev/md0
		elif [ "$FS" == "btrfs" ]; then
			mkfs.btrfs /dev/md0
		fi
	else
		# need to reassemble the raid
		mdadm --assemble /dev/md0 --force $PARTITIONS
	fi

	if [ "$FS" == "ext4" ]; then
		mount -t ext4 /dev/md0 /mnt -o noatime,data=writeback,barrier=0,nobh
	elif [ "$FS" == "xfs" ]; then
		mount -t xfs /dev/md0 /mnt -o noatime,nodirspread
	elif [ "$FS" == "btrfs" ]; then
		mount -t btrfs /dev/md0 /mnt -o noatime
	fi

	if [ "$LARGE_PAGE" == "no" ]; then
		echo 4096 > /sys/block/md0/md/stripe_cache_size
	else
		echo 256 > /sys/block/md0/md/stripe_cache_size
	fi
	/bin/echo -e 4096 > /sys/block/md0/queue/read_ahead_kb

	for drive in `echo $DRIVES`; do echo noop > /sys/block/sd${drive}/queue/scheduler; done
	echo 4 > /proc/sys/vm/dirty_ratio
	echo 2 > /proc/sys/vm/dirty_background_ratio
	set +o verbose
elif [ "$TOPOLOGY" == "rd1" ]; then
	echo -ne "\nStarting RAID1 build\n"
	for drive in `echo $DRIVES`; do PARTITIONS="${PARTITIONS} /dev/sd${drive}${PARTNUM}"; done

	set -o verbose

	for drive in `echo $DRIVES`; do echo -e 1024 > /sys/block/sd${drive}/queue/read_ahead_kb; done

	if [ "$MKFS" == "yes" ]; then
		[ -e /dev/md0 ] && mdadm --manage --stop /dev/md0

		for partition in `echo $DRIVES`; do mdadm --zero-superblock /dev/sd${partition}1; done
		echo "y" | mdadm --create -c 128 /dev/md0 --level=1 -n 2 --force $PARTITIONS
		if [ "$LARGE_PAGE" == "no" ]; then
		    echo 4096 > /sys/block/md0/md/stripe_cache_size
		else
		    echo 256 > /sys/block/md0/md/stripe_cache_size
		fi

		if [ "$FS" == "ext4" ];	then
			mkfs.ext4 -j -m 0 -T largefile -b 4096 -E stride=32,stripe-width=32 -F /dev/md0
		elif [ "$FS" == "btrfs" ]; then
			mkfs.btrfs -m raid1 -d raid1 /dev/md0
		fi
	else
		# need to reassemble the raid
		mdadm --assemble /dev/md0 --force $PARTITIONS
	fi

	if [ "$FS" == "ext4" ]; then
		mount -t ext4 /dev/md0 /mnt -o noatime,data=writeback,barrier=0
	elif [ "$FS" == "btrfs" ]; then
		mount -t btrfs /dev/md0 /mnt -o noatime
	fi
	
	for drive in `echo $DRIVES`; do /bin/echo -e 1024 > /sys/block/sd${drive}/queue/read_ahead_kb; done
	/bin/echo -e 2048 > /sys/block/md0/queue/read_ahead_kb
	echo 4 > /proc/sys/vm/dirty_ratio
	echo 2 > /proc/sys/vm/dirty_background_ratio
	set +o verbose
elif [ "$TOPOLOGY" == "rd0" ]; then
	echo -ne "\nStarting RAID0 build\n"

	for drive in `echo $DRIVES`; do PARTITIONS="${PARTITIONS} /dev/sd${drive}${PARTNUM}"; done

	set -o verbose

	for drive in `echo $DRIVES`; do echo -e 1024 > /sys/block/sd${drive}/queue/read_ahead_kb; done

	if [ "$MKFS" == "yes" ]; then
		if [ -e /dev/md0 ]; then
			mdadm --manage --stop /dev/md0
		fi
		for partition in `echo $DRIVES`; do mdadm --zero-superblock /dev/sd${partition}1; done
		if [ "$SSD" == "no" ]; then
			echo "y" | mdadm --create -c 256 /dev/md0 --level=0 -n 2 --force $PARTITIONS
		else
			echo "y" | mdadm --create -c 256 /dev/md0 --level=0 -n 2 --force $PARTITIONS
		fi
		echo 256 > /sys/block/md0/md/stripe_cache_size
		if [ "$FS" == "ext4" ]; then
			if [ "$SSD" == "no" ]; then
				mkfs.ext4 -j -m 0 -T largefile -b 4096 -E stride=64,stripe-width=128 -F /dev/md0
			else
				mkfs.ext4 -j -m 0 -T largefile -b 4096 -E stride=64,stripe-width=128 -F /dev/md0
			fi
		elif [ "$FS" == "btrfs" ]; then
				mkfs.btrfs -m raid0 -d raid0 /dev/md0
			fi
	fi
	mdadm --assemble /dev/md0 --force $PARTITIONS
	if [ "$FS" == "ext4" ]; then
		mount -t ext4 /dev/md0 /mnt -o noatime,data=writeback,barrier=0
	elif [ "$FS" == "btrfs" ]; then
		mount -t ext4 /dev/md0 /mnt -o noatime
	fi

	for drive in `echo $DRIVES`; do /bin/echo -e 1024 > /sys/block/sd${drive}/queue/read_ahead_kb; done
	echo -e 4096 > /sys/block/md0/queue/read_ahead_kb

	set +o verbose
elif [ "$TOPOLOGY" == "sd" ]; then
	PARTITIONS="/dev/sd${DRIVES}${PARTNUM}"

	set -o verbose

	echo -e 1024 > /sys/block/sd${DRIVES}/queue/read_ahead_kb

	if [ "$FS" == "fat32" ]; then
		echo -ne "\nStarting SINGLE drive (${PARTITIONS}) fat32\n"
		# wait for DAS recongnition by udev
		sleep 5
		if [ "$MKFS" == "yes" ]; then
			[ -e /dev/md0 ] && mdadm --manage --stop /dev/md0
			mkdosfs -s 64 -S 4096 -F 32 $PARTITIONS
		fi
		mount -t vfat -o umask=000,rw,noatime,nosuid,nodev,noexec,async $PARTITIONS /mnt
	elif [ "$FS" == "ext4" ]; then
		echo -ne "\nStarting SINGLE drive (${PARTITIONS}) ext4\n"
		if [ "$MKFS" == "yes" ]; then
			[ -e /dev/md0 ] && mdadm --manage --stop /dev/md0
			
			mkfs.ext4 -j -m 0 -b 4096 -O large_file,extents -F $PARTITIONS
		fi
		mount -t ext4 $PARTITIONS /mnt -o noatime,data=writeback,barrier=0
	elif [ "$FS" == "btrfs" ]; then
		echo -ne "\nStarting SINGLE drive (${PARTITIONS}) btrfs\n"
		if [ "$MKFS" == "yes" ]; then
			[ -e /dev/md0 ] && mdadm --manage --stop /dev/md0
			mkfs.btrfs -m single -d single $PARTITIONS
		fi

		mount -t btrfs $PARTITIONS /mnt -o noatime
	else
		echo -ne "\nStarting SINGLE drive (${PARTITIONS}) xfs\n"
		if [ "$MKFS" == "yes" ]; then
			[ -e /dev/md0 ] && mdadm --manage --stop /dev/md0
			mkfs.xfs -f $PARTITIONS
		fi
		mount -t xfs $PARTITIONS /mnt -o noatime,nodirspread
	fi
	set +o verbose
fi

echo -ne "\n==================================================\n"
sleep 1
echo -ne "\n==================================================\n"

mkdir -p /mnt/public
chmod 777 /mnt/
chmod 777 /mnt/public
for (( i=0; i<4; i++ )); do
	mkdir -p /mnt/public/share$i
	chmod 777 /mnt/public/share$i
done

# Samba
if [ "$FS" != "NONE" ]; then
	if [[ -e "$(which smbd)" && -e "$(which nmbd)" ]]; then
		chmod 0755 /var/lock
		rm -rf /etc/smb.conf
		touch  /etc/smb.conf
		echo '[global]' 				>>  /etc/smb.conf
		echo ' netbios name = debian-armada'		>>  /etc/smb.conf
		echo ' workgroup = WORKGROUP'			>>  /etc/smb.conf
		echo ' server string = debian-armada'		>>  /etc/smb.conf
		echo ' encrypt passwords = yes'			>>  /etc/smb.conf
		echo ' security = user'				>>  /etc/smb.conf
		echo ' map to guest = bad password'		>>  /etc/smb.conf
		echo ' use mmap = yes'				>>  /etc/smb.conf
		echo ' use sendfile = yes'			>>  /etc/smb.conf
		echo ' dns proxy = no'				>>  /etc/smb.conf
		echo ' max log size = 200'			>>  /etc/smb.conf
		echo ' log level = 0'				>>  /etc/smb.conf
		echo ' socket options = IPTOS_LOWDELAY TCP_NODELAY SO_SNDBUF=131072 SO_RCVBUF=131072' >>  /etc/smb.conf
		echo ' local master = no'			>>  /etc/smb.conf
		echo ' dns proxy = no'				>>  /etc/smb.conf
		echo ' ldap ssl = no'				>>  /etc/smb.conf
		echo ' create mask = 0666'			>>  /etc/smb.conf
		echo ' directory mask = 0777'			>>  /etc/smb.conf
		echo ' show add printer wizard = No'		>>  /etc/smb.conf
		echo ' printcap name = /dev/null'		>>  /etc/smb.conf
		echo ' load printers = no'			>>  /etc/smb.conf
		echo ' disable spoolss = Yes'			>>  /etc/smb.conf
		echo ' max xmit = 131072'			>>  /etc/smb.conf
		echo ' disable netbios = yes'			>>  /etc/smb.conf
		echo ' csc policy = disable'			>>  /etc/smb.conf
		if [[ "$FS" == "xfs" || "$FS" == "btrfs" ]]; then
			# crash identified with these FS
			echo ' min receivefile size = 128k'	>>  /etc/smb.conf
			echo ' strict allocate = yes'		>>  /etc/smb.conf
		else
			echo ' min receivefile size = 128k'	>>  /etc/smb.conf
			echo ' strict allocate = yes'		>>  /etc/smb.conf
		fi
		echo ' allocation roundup size = 10485760'	>>  /etc/smb.conf
		echo ''						>>  /etc/smb.conf
		echo '[public]'					>>  /etc/smb.conf
		echo ' comment = my public share'		>>  /etc/smb.conf
		echo ' path = /mnt/public'			>>  /etc/smb.conf
		echo ' writeable = yes'				>>  /etc/smb.conf
		echo ' printable = no'				>>  /etc/smb.conf
		echo ' public = yes'				>>  /etc/smb.conf
		echo ''						>>  /etc/smb.conf

		echo -ne "* Starting Samba daemons\n"
		rm -rf /var/log/log.smbd
		rm -rf /var/log/log.nmbd
		$(which nmbd) -D -s /etc/smb.conf
		$(which smbd) -D -s /etc/smb.conf
		sleep 1
		echo -ne ". Done\n"
	else
		sleep 1
		echo "x Samba skipped"
	fi
fi
echo -ne "\n==================================================\n"

if [[ "$ARCH" == "armv6l" || "$ARCH" == "armv7l" ]]; then
	if [ "$CPU_COUNT" = "4" ]; then
		set -o verbose
		if [ "$TOPOLOGY" == "rd1" ]; then
			taskset  -p 0x08 `pidof md0_raid1`
		elif [ "$TOPOLOGY" == "rd5" ]; then
			taskset  -p 0x0f `pidof md0_raid5`
		fi

		taskset  -p 0x0f `pidof kswapd0`
		taskset  -p 0x0f `pidof flush-9:0`
		# ETH
		#echo 1 > /proc/irq/8/smp_affinity
		#echo 2 > /proc/irq/10/smp_affinity
		#echo 4 > /proc/irq/12/smp_affinity
		#echo 8 > /proc/irq/14/smp_affinity
		# XORs
		echo f > /proc/irq/51/smp_affinity
		echo f > /proc/irq/52/smp_affinity
		# SATA
		echo f > /proc/irq/55/smp_affinity
		#PEX
		#echo f > /proc/irq/61/smp_affinity
		set +o verbose
		echo -ne "set samba task affinity:\n"
		#read -p "Press Enter when ready"
		#echo -ne "  taskset  -p 0x02 \`ps -ef | grep smbd | grep nobody | tr -s ' ' ',' | cut -d , -f 2\`"
		for i in $(ps | grep smbd | grep nobody | tr -s ' ' ',' | cut -d , -f 2); do
			taskset  -p 0x02 $i
		done
	elif [ "$CPU_COUNT" == "2" ]; then
		set -o verbose
		#echo 3  > /proc/irq/8/smp_affinity
		echo 3  > /proc/irq/51/smp_affinity
		echo 3  > /proc/irq/52/smp_affinity
		echo 2  > /proc/irq/99/smp_affinity
		set +o verbose
		echo -ne "set samba task affinity:\n"
		echo -ne "  taskset  -p 0x02 \`ps -ef | grep smbd | grep nobody | tr -s ' ' ',' | cut -d , -f 2\`"
	fi
fi

case "$TOPOLOGY" in
	rd1 | rd5)
		#watch "cat /proc/mdstat|grep finish"
	;;
esac

echo -e -n \\a
echo -ne "\n==============================================done\n"
