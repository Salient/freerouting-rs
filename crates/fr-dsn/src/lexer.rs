//! Tolerant s-expression tokenizer for Specctra DSN/SES/RTE files.
//!
//! The Specctra format is a parenthesized s-expression language. Tokens are: `(`, `)`,
//! bare identifiers/numbers, and quoted strings (the quote char is declared by
//! `(string_quote ...)`, default `"`). Altium also encodes spaces/parens inside names
//! as `~SP~`/`~LP~`/`~RP~`, so a bare token can contain those literally.
//!
//! The lexer is deliberately permissive: it never panics on malformed input, just
//! produces the best token stream it can (the parser layer decides what to tolerate).

#[derive(Clone, PartialEq, Eq, Debug)]
pub enum Token {
    /// `(`
    Open,
    /// `)`
    Close,
    /// A bare or quoted atom (identifier, number, or string), quote stripped.
    Atom(String),
}

/// Tokenize DSN source. `quote_char` is the active string-quote character (default `"`).
/// Returns the full token vector (files are small enough — low MBs — to hold in memory).
pub fn tokenize(src: &str, quote_char: char) -> Vec<Token> {
    let mut out = Vec::new();
    let mut chars = src.chars().peekable();
    while let Some(&c) = chars.peek() {
        match c {
            '(' => {
                chars.next();
                out.push(Token::Open);
            }
            ')' => {
                chars.next();
                out.push(Token::Close);
            }
            c if c.is_whitespace() => {
                chars.next();
            }
            c if c == quote_char => {
                // quoted string: consume until the closing quote char
                chars.next();
                let mut s = String::new();
                while let Some(&qc) = chars.peek() {
                    if qc == quote_char {
                        chars.next();
                        break;
                    }
                    s.push(qc);
                    chars.next();
                }
                out.push(Token::Atom(s));
            }
            _ => {
                // bare atom: read until whitespace or a paren
                let mut s = String::new();
                while let Some(&bc) = chars.peek() {
                    if bc.is_whitespace() || bc == '(' || bc == ')' {
                        break;
                    }
                    s.push(bc);
                    chars.next();
                }
                if !s.is_empty() {
                    out.push(Token::Atom(s));
                }
            }
        }
    }
    out
}

/// Scan ahead for `(string_quote X)` so we can re-tokenize with the right quote char if
/// it differs from the default. Returns the declared quote char, if any.
pub fn detect_string_quote(src: &str) -> Option<char> {
    // cheap textual scan; the parser doesn't need perfect nesting here.
    let idx = src.find("(string_quote")?;
    let rest = &src[idx + "(string_quote".len()..];
    rest.chars().find(|c| !c.is_whitespace() && *c != ')')
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn basic_scopes() {
        let toks = tokenize("(pcb foo (resolution mil 10000))", '"');
        assert_eq!(toks[0], Token::Open);
        assert_eq!(toks[1], Token::Atom("pcb".into()));
        assert_eq!(toks[2], Token::Atom("foo".into()));
        assert_eq!(toks[3], Token::Open);
        assert_eq!(toks[4], Token::Atom("resolution".into()));
        assert_eq!(*toks.last().unwrap(), Token::Close);
    }

    #[test]
    fn quoted_names_with_specials() {
        // a quoted net name containing spaces and parens
        let toks = tokenize("(net \"Net-(C2-Pad1)\")", '"');
        assert_eq!(toks[2], Token::Atom("Net-(C2-Pad1)".into()));
    }

    #[test]
    fn altium_sp_encoded_bare_atom() {
        // Altium writes spaces as ~SP~ in bare atoms
        let toks = tokenize("(net UART~SP~TO~SP~PINE)", '"');
        assert_eq!(toks[2], Token::Atom("UART~SP~TO~SP~PINE".into()));
    }

    #[test]
    fn tolerates_unbalanced_without_panic() {
        // missing close paren must not panic, just produce tokens
        let toks = tokenize("(structure (layer TopLayer ", '"');
        assert!(toks.contains(&Token::Atom("TopLayer".into())));
    }

    #[test]
    fn detect_quote_char() {
        assert_eq!(detect_string_quote("(parser (string_quote \"))"), Some('"'));
        assert_eq!(detect_string_quote("(pcb foo)"), None);
    }
}
