import { useState } from "react";
import type { GeocodeResult } from "../lib/geocode";

interface SearchBarProps {
  onSearch: (query: string) => Promise<void>;
  searching: boolean;
  searchResults: GeocodeResult[];
  onPickResult: (r: GeocodeResult) => void;
}

export default function SearchBar(props: SearchBarProps) {
  const [query, setQuery] = useState("");

  return (
    <div className="search-float">
      <form
        className="search-pill"
        onSubmit={(e) => {
          e.preventDefault();
          props.onSearch(query);
        }}
      >
        <span className="search-icon" aria-hidden>
          ⌕
        </span>
        <input
          type="text"
          placeholder="Shaw Ave & Pioneer Ave, Fresno CA"
          value={query}
          onChange={(e) => setQuery(e.target.value)}
        />
        <button type="submit" disabled={props.searching}>
          {props.searching ? "…" : "Go"}
        </button>
      </form>
      {props.searchResults.length > 0 && (
        <ul className="results results-float">
          {props.searchResults.map((r, i) => (
            <li key={i}>
              <button onClick={() => props.onPickResult(r)}>{r.displayName}</button>
            </li>
          ))}
        </ul>
      )}
    </div>
  );
}
