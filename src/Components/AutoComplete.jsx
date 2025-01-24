import React from 'react';
import Autosuggest from 'react-autosuggest';
import {searchWiki} from '../js/serverApi.js';

function AutoComplete() {
  const [inputValue, setInputValue] = React.useState('');
  const [suggestions, setSuggestions] = React.useState([]);
  const onChangeInput = React.useCallback((event, {newValue, method}) => {
    setInputValue(newValue);
  }, [])
  const requestSearch = React.useCallback(({value}) => {
    searchWiki(value)
    .then(result => {
      setSuggestions(result.slice(0,100));
    })
  }, [])
  const renderSuggestion = React.useCallback((suggestion, {query}) => {
    const suggestionText = suggestion.text;
    return (
      <div>{suggestionText}</div>
    )

  }, [])
  const inputProps = {
    placeholder: '검색할 사람',
    value: inputValue,
    onChange: onChangeInput
  }
  return (
    <Autosuggest
      suggestions={suggestions}
      onSuggestionsFetchRequested={requestSearch}
      renderSuggestion={renderSuggestion}
      inputProps={inputProps}

    ></Autosuggest>
  )
}

export default React.memo(AutoComplete)