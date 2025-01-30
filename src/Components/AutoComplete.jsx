import React from 'react';
import Autosuggest from 'react-autosuggest';
import {searchWiki} from '../js/serverApi.js';
import './autosuggest.css'
import styled from 'styled-components';
import _ from 'lodash';

const Container = styled.div`
  display: flex;
  align-items: center;
`
const LoadingText = styled.div`
  color: white;
  margin-left: 10px;
`


function AutoComplete(props) {
  const {onSuggestSelected} = props;
  const [isLoading, setIsLoading] = React.useState(false);
  const [inputValue, setInputValue] = React.useState('');
  const [suggestions, setSuggestions] = React.useState([]);
  const onChangeInput = React.useCallback((event, {newValue, method}) => {
    setInputValue(newValue);
  }, [])
  const requestSearch = React.useCallback((value) => {
    setIsLoading(true);
    searchWiki(value)
    .then(result => {
      setIsLoading(false);
      setSuggestions(result.slice(0,100));
    })
    .catch(err => {
      setIsLoading(false);
      setSuggestions([]);
    })
  }, [])
  const debouncedReqSearch = React.useMemo((value) => {
    return _.debounce((value) => requestSearch(value), 200)
  }, [])
  const requestSuggestion = ({value}) => {
    debouncedReqSearch(value);
  }
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
    placeholder: '네트워크에 추가',
    value: inputValue,
    onChange: onChangeInput
  }
  return (
    <Container>
      <Autosuggest
        suggestions={suggestions}
        onSuggestionsFetchRequested={requestSuggestion}
        onSuggestionsClearRequested={clearSuggestion}
        renderSuggestion={renderSuggestion}
        getSuggestionValue={getSuggestionValue}
        // alwaysRenderSuggestions={true}
        inputProps={inputProps}

      ></Autosuggest>
      {isLoading && (
        <LoadingText>searching...</LoadingText>
      )}
    </Container>
  )
}

export default React.memo(AutoComplete)