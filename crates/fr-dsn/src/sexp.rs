//! A generic s-expression tree built from the token stream, plus tolerant navigation
//! helpers. The DSN reader walks this tree rather than the raw tokens, which keeps the
//! "skip unknown / malformed scope" behavior simple and local.

use crate::lexer::{tokenize, Token};

/// Parse a list whose opening `(` was already consumed; `pos` points at the first
/// element inside. Consumes through the matching `)` (or end-of-input if unbalanced).
fn parse_list(toks: &[Token], pos: &mut usize) -> Sexp {
    let mut items = Vec::new();
    while *pos < toks.len() {
        match &toks[*pos] {
            Token::Open => {
                *pos += 1;
                items.push(parse_list(toks, pos));
            }
            Token::Close => {
                *pos += 1;
                break;
            }
            Token::Atom(a) => {
                items.push(Sexp::Atom(a.clone()));
                *pos += 1;
            }
        }
    }
    Sexp::List(items)
}

/// An s-expression node: either an atom (leaf) or a list (scope).
#[derive(Clone, PartialEq, Eq, Debug)]
pub enum Sexp {
    Atom(String),
    List(Vec<Sexp>),
}

impl Sexp {
    /// Parse a full DSN source string into a top-level list of nodes. Unbalanced input
    /// is tolerated: unclosed lists are closed at end-of-input.
    pub fn parse(src: &str, quote_char: char) -> Sexp {
        let toks = tokenize(src, quote_char);
        let mut pos = 0;
        let mut top = Vec::new();
        while pos < toks.len() {
            match &toks[pos] {
                Token::Open => {
                    pos += 1;
                    top.push(parse_list(&toks, &mut pos));
                }
                Token::Atom(a) => {
                    top.push(Sexp::Atom(a.clone()));
                    pos += 1;
                }
                Token::Close => {
                    pos += 1; // stray close, ignore
                }
            }
        }
        // A well-formed DSN is a single top-level (pcb ...) list; return it directly if so.
        if top.len() == 1 {
            top.pop().unwrap()
        } else {
            Sexp::List(top)
        }
    }

    pub fn as_atom(&self) -> Option<&str> {
        match self {
            Sexp::Atom(s) => Some(s.as_str()),
            _ => None,
        }
    }

    pub fn as_list(&self) -> Option<&[Sexp]> {
        match self {
            Sexp::List(v) => Some(v.as_slice()),
            _ => None,
        }
    }

    /// The keyword of a list scope: the first element if it is an atom (e.g. "pcb").
    pub fn head(&self) -> Option<&str> {
        self.as_list()?.first()?.as_atom()
    }

    /// All direct child scopes (lists) whose head equals `kw`. The keyword is owned by
    /// the returned iterator so its lifetime is tied only to `self`.
    pub fn children<'a>(&'a self, kw: &str) -> impl Iterator<Item = &'a Sexp> + 'a {
        let slice = self.as_list().unwrap_or(&[]);
        let want = kw.to_string();
        slice.iter().filter(move |c| c.head() == Some(want.as_str()))
    }

    /// The first direct child scope with head `kw`.
    pub fn child(&self, kw: &str) -> Option<&Sexp> {
        self.children(kw).next()
    }

    /// Atom arguments of this scope (the atoms after the head keyword).
    pub fn atom_args(&self) -> Vec<&str> {
        let slice = self.as_list().unwrap_or(&[]);
        slice.iter().skip(1).filter_map(|c| c.as_atom()).collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_nested_and_navigate() {
        let s = Sexp::parse("(pcb board.dsn (resolution MIL 10000) (structure (layer TopLayer)))", '"');
        assert_eq!(s.head(), Some("pcb"));
        let res = s.child("resolution").unwrap();
        assert_eq!(res.atom_args(), vec!["MIL", "10000"]);
        let structure = s.child("structure").unwrap();
        assert_eq!(structure.child("layer").unwrap().atom_args(), vec!["TopLayer"]);
    }

    #[test]
    fn multiple_children() {
        let s = Sexp::parse("(structure (layer A) (layer B) (layer C))", '"');
        let layers: Vec<_> = s.children("layer").collect();
        assert_eq!(layers.len(), 3);
    }

    #[test]
    fn tolerates_unbalanced() {
        // unclosed structure scope must still parse without panic
        let s = Sexp::parse("(pcb x (structure (layer TopLayer ", '"');
        assert_eq!(s.head(), Some("pcb"));
        assert!(s.child("structure").is_some());
    }
}
