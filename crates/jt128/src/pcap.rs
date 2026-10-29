//! Minimal reader for classic libpcap files (not pcapng) holding Ethernet
//! frames. Yields the UDP datagrams inside, skipping everything else.

use std::io::{self, Read};

/// One UDP datagram from a capture.
#[derive(Debug)]
pub struct UdpPacket<'a> {
    /// Capture time on the recording host, seconds.
    pub capture_time: f64,
    pub src_port: u16,
    pub dst_port: u16,
    pub payload: &'a [u8],
}

pub struct PcapReader<R> {
    reader: R,
    nanos: bool,
    big_endian: bool,
    record: Vec<u8>,
}

fn invalid(msg: &str) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidData, msg.to_string())
}

impl<R: Read> PcapReader<R> {
    pub fn new(mut reader: R) -> io::Result<Self> {
        let mut header = [0u8; 24];
        reader.read_exact(&mut header)?;
        let magic_le = u32::from_le_bytes(header[0..4].try_into().unwrap());
        let (nanos, big_endian) = match magic_le {
            0xA1B2_C3D4 => (false, false),
            0xA1B2_3C4D => (true, false),
            0xD4C3_B2A1 => (false, true),
            0x4D3C_B2A1 => (true, true),
            _ => return Err(invalid("not a classic pcap file (pcapng is not supported)")),
        };
        let s = Self {
            reader,
            nanos,
            big_endian,
            record: Vec::new(),
        };
        let linktype = s.u32(&header[20..24]);
        if linktype != 1 {
            return Err(invalid(
                "only Ethernet (link type 1) captures are supported",
            ));
        }
        Ok(s)
    }

    fn u32(&self, b: &[u8]) -> u32 {
        let a: [u8; 4] = b.try_into().unwrap();
        if self.big_endian {
            u32::from_be_bytes(a)
        } else {
            u32::from_le_bytes(a)
        }
    }

    /// The next UDP datagram, or `None` at the end of the file.
    pub fn next_udp(&mut self) -> io::Result<Option<UdpPacket<'_>>> {
        loop {
            let mut rec = [0u8; 16];
            match self.reader.read_exact(&mut rec) {
                Ok(()) => {}
                Err(e) if e.kind() == io::ErrorKind::UnexpectedEof => return Ok(None),
                Err(e) => return Err(e),
            }
            let secs = self.u32(&rec[0..4]);
            let frac = self.u32(&rec[4..8]);
            let incl = self.u32(&rec[8..12]) as usize;
            self.record.resize(incl, 0);
            self.reader.read_exact(&mut self.record)?;
            let capture_time = secs as f64 + frac as f64 * if self.nanos { 1e-9 } else { 1e-6 };
            if let Some((src_port, dst_port, range)) = udp_in_ethernet(&self.record) {
                return Ok(Some(UdpPacket {
                    capture_time,
                    src_port,
                    dst_port,
                    payload: &self.record[range],
                }));
            }
        }
    }
}

/// Locates the UDP payload in an Ethernet frame (IPv4, optional 802.1Q tag).
fn udp_in_ethernet(f: &[u8]) -> Option<(u16, u16, std::ops::Range<usize>)> {
    let be16 = |o: usize| f.get(o..o + 2).map(|b| u16::from_be_bytes([b[0], b[1]]));
    let mut off = 12;
    let mut ethertype = be16(off)?;
    if ethertype == 0x8100 {
        off += 4;
        ethertype = be16(off)?;
    }
    if ethertype != 0x0800 {
        return None;
    }
    let ip = off + 2;
    let ihl = usize::from(*f.get(ip)? & 0x0F) * 4;
    if *f.get(ip + 9)? != 17 {
        return None;
    }
    let udp = ip + ihl;
    let src = be16(udp)?;
    let dst = be16(udp + 2)?;
    let len = usize::from(be16(udp + 4)?);
    let end = (udp + len).min(f.len());
    (len >= 8).then_some((src, dst, udp + 8..end))
}
