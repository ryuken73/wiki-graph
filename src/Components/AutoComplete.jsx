import React from 'react';
import Autosuggest from 'react-autosuggest';
import {searchWiki} from '../js/serverApi.js';
import './autosuggest.css'


function AutoComplete(props) {
  const {onSuggestSelected} = props;
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
  const clearSuggestion = React.useCallback(() => {
      setSuggestions([]);
  }, [])
  const getSuggestionValue = React.useCallback((suggestion) => {
    console.log('selected:', suggestion);
    const node = {
      ...suggestion,
      backlinkId: suggestion.backlink_id
    }
    const isNodeContent = suggestion.content_id !== null;
    onSuggestSelected(node.id, isNodeContent);
    return suggestion.text;
  }, []);
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
      onSuggestionsClearRequested={clearSuggestion}
      renderSuggestion={renderSuggestion}
      getSuggestionValue={getSuggestionValue}
      // alwaysRenderSuggestions={true}
      inputProps={inputProps}

    ></Autosuggest>
  )
}

export default React.memo(AutoComplete)