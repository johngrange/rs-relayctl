//
// #![no_std]

const MAX_CMD_LEN: usize = 64;

use core::str::FromStr;

use heapless::{String, Vec};
const MAX_DEVICE_NAME_LEN: usize = 10;

#[derive(Debug, PartialEq, Eq)]
pub enum Cmd {
    On(String<MAX_DEVICE_NAME_LEN>),
    Off(String<MAX_DEVICE_NAME_LEN>),
    GetAll,
    Get(String<MAX_DEVICE_NAME_LEN>),
}

pub struct CmdParser {
    buf: Vec<u8, MAX_CMD_LEN>,
    cmd_str: String<MAX_CMD_LEN>,
}

impl CmdParser {
    pub fn new() -> CmdParser {
        CmdParser {
            buf: Vec::new(),
            cmd_str: String::new(),
        }
    }

    pub fn parse_str(&mut self, s: &str) -> Result<Option<Cmd>, CmdError> {
        self.parse_bytes(s.as_bytes())
    }

    pub fn parse_bytes(&mut self, b: &[u8]) -> Result<Option<Cmd>, CmdError> {
        if self.buf.extend_from_slice(b).is_err() {
            return Err(CmdError::BufferTooLong);
        }
        let mut s: String<MAX_CMD_LEN> = match String::from_utf8(self.buf.clone()) {
            Ok(s) => s,
            Err(_) => {
                return Err(CmdError::CannotParseString);
            }
        };
        s.make_ascii_uppercase();

        // Parsed buffer, so no longer nee
        if self.cmd_str.push_str(s.as_str()).is_err() {
            return Err(CmdError::StringTooLong);
        }
        self.buf.clear();

        self.next_cmd()
    }

    pub fn next_cmd(&mut self) -> Result<Option<Cmd>, CmdError> {
        if self.cmd_str.contains("\n") {
            let mut lines = self.cmd_str.lines();
            let line1 = lines.nth(0).unwrap(); // OK as we've checked it should get this
            let line: String<MAX_CMD_LEN> = String::from_str(line1).unwrap();

            let remainder = self.cmd_str[line1.len()..].trim_start_matches('\n');

            self.cmd_str = String::from_str(remainder).unwrap();

            if line.starts_with(ON_CMD) {
                let (_, dev) = line.split_once(' ').unwrap(); // We know we have a space, we've just checked!

                let dev_s: String<MAX_DEVICE_NAME_LEN> = match String::from_str(&dev.trim()) {
                    Ok(s) => s,
                    Err(_) => return Err(CmdError::StringTooLong),
                };
                return Ok(Some(Cmd::On(dev_s)));
            }
            if line.starts_with(OFF_CMD) {
                let (_, dev) = line.split_once(' ').unwrap(); // We know we have a space, we've just checked!

                let dev_s: String<MAX_DEVICE_NAME_LEN> = match String::from_str(&dev.trim()) {
                    Ok(s) => s,
                    Err(_) => return Err(CmdError::StringTooLong),
                };
                return Ok(Some(Cmd::Off(dev_s)));
            }
            if line.starts_with(GET_CMD) {
                let (_, dev) = line.split_once(' ').unwrap(); // We know we have a space, we've just checked!

                let dev_s: String<MAX_DEVICE_NAME_LEN> = match String::from_str(&dev.trim()) {
                    Ok(s) => s,
                    Err(_) => return Err(CmdError::StringTooLong),
                };
                return Ok(Some(Cmd::Get(dev_s)));
            }
            if line.starts_with(GETALL_CMD) {
                return Ok(Some(Cmd::GetAll));
            }
        }
        Ok(None)
    }
}

const ON_CMD: &str = "ON ";
const OFF_CMD: &str = "OFF ";
const GET_CMD: &str = "GET ";
const GETALL_CMD: &str = "GETALL";

#[derive(Debug, Clone)]
pub enum CmdError {
    CannotParseString,
    BufferTooLong,
    StringTooLong,
    UnexpectedError,
}

#[test]
fn test_parse_sunny() {
    use core::str::FromStr;
    let mut p = CmdParser::new();

    assert!(p.parse_bytes("O".as_bytes()).unwrap().is_none());
    assert!(p.parse_bytes("N".as_bytes()).unwrap().is_none());
    assert!(p.parse_bytes(" ".as_bytes()).unwrap().is_none());
    assert!(p.parse_bytes("1".as_bytes()).unwrap().is_none());
    let c = p.parse_bytes("\n".as_bytes());
    assert!(c.as_ref().is_ok());
    assert!(c.as_ref().unwrap().is_some());
    let s: String<MAX_DEVICE_NAME_LEN> = String::from_str("1").unwrap();
    match c.unwrap() {
        Some(cmd) => {
            assert_eq!(cmd, Cmd::On(s))
        }
        None => {
            assert!(false)
        }
    }
    let c2 = p.parse_bytes("OFF 32\n".as_bytes());
    assert!(c2.as_ref().is_ok());
    assert!(c2.as_ref().unwrap().is_some());
    let cmd = c2.unwrap().unwrap();
    assert_eq!(cmd, Cmd::Off(String::from_str("32").unwrap()));

    let c3 = p.parse_bytes("GETALL\n".as_bytes());
    assert!(c3.as_ref().is_ok());
    assert!(c3.as_ref().unwrap().is_some());
    let cmd3 = c3.unwrap().unwrap();
    assert_eq!(cmd3, Cmd::GetAll);

    let c4 = p.parse_bytes("GET LED\n".as_bytes());
    assert!(c4.as_ref().is_ok());
    assert!(c4.as_ref().unwrap().is_some());
    let cmd4 = c4.unwrap().unwrap();
    assert_eq!(cmd4, Cmd::Get(String::from_str("LED").unwrap()));
}

#[test]
fn test_trim_cr() {
    use core::str::FromStr;
    let mut p = CmdParser::new();
    let c = p.parse_bytes("GET LED\r\n".as_bytes());
    assert!(c.as_ref().is_ok());
    assert!(c.as_ref().unwrap().is_some());
    let cmd = c.unwrap().unwrap();
    assert_eq!(cmd, Cmd::Get(String::from_str("LED").unwrap()));
}

#[test]
fn test_cmd_case() {
    use core::str::FromStr;
    let mut p = CmdParser::new();
    let c = p.parse_bytes("GeT lEd\n".as_bytes());
    assert!(c.as_ref().is_ok());
    assert!(c.as_ref().unwrap().is_some());
    let cmd = c.unwrap().unwrap();
    assert_eq!(cmd, Cmd::Get(String::from_str("LED").unwrap()));
}

#[test]
fn test_mult_in_one_parse() {
    use core::str::FromStr;
    let mut p = CmdParser::new();
    let c1 = p.parse_str("GET LED\nGETALL\n");
    assert!(c1.as_ref().is_ok());
    assert!(c1.as_ref().unwrap().is_some());
    let cmd1 = c1.unwrap().unwrap();
    assert_eq!(cmd1, Cmd::Get(String::from_str("LED").unwrap()));

    let c2 = p.next_cmd();
    assert!(c2.as_ref().is_ok());
    assert!(c2.as_ref().unwrap().is_some());
    let cmd2 = c2.unwrap().unwrap();
    assert_eq!(cmd2, Cmd::GetAll);

    let c3 = p.next_cmd();
    assert!(c3.as_ref().is_ok());
    assert!(c3.as_ref().unwrap().is_none());
}
