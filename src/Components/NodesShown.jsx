import React from 'react';
import styled from 'styled-components';
import MiniSearch from 'minisearch';
import hangul from 'hangul-js';

const Container = styled.div`
  display: flex;
  flex-direction: column;
  height: 100%;
  /* border: 2px maroon solid; */
  min-width: 150px;
  max-width: 150px;
  user-select: none;
`
const Rows = styled.div`
  height: 100%;
  margin-top: 3px;
  padding-right: 3px;
  padding-left: 3px;
  /* overflow-y: scroll; */
  &::-webkit-scrollbar {
    width: 10px;
  };
  &::-webkit-scrollbar-thumb {
    background-color: #9b6a2f;
  }
  &::-webkit-scrollbar-track {
    background-color: black;
  }
`
const RowContainer = styled.div`
  padding: 3px;
  background-color: black;
  margin-bottom: 3px;
`
const CCTV = styled.div`
  display: flex;
  font-size: 12px;
  font-weight: 100;
  color: yellow;
`
const Action = styled.div`
  margin-left: 3px;
`
const Title = styled(Action)`
  /* color: ${(props) => props.action === 'del' && 'lightgrey'}; */
  cursor: pointer;
  width: 100%;
  white-space: nowrap;
  text-overflow: ellipsis;
  overflow: hidden;
  &:hover {
    color: white;
  };
`
const DelButton = styled(Action)`
  color: red;
  margin-left: auto;
  font-weight: 200;
  cursor: pointer;
  &:hover {
    color: white;
  };
`
const TimeStamp = styled(Action)`
  font-size: 11px;
  color: #d73232;
`
const HistoryCount = styled.div`
  font-size: 12px;
  margin-top: auto;
  padding: 5px;
  color: yellow;
  font-weight: 200;
`

const SEARCH_OPTION = {
  prefix: true,
  fields: ['json.title'],
  fuzzy: 0.3
};


function NodesShown(props) {
  // eslint-disable-next-line react/prop-types
  const {nodesExpanded, removeNode} = props;
  const [cctvHistory, setHistory] = React.useState([]);
  const [cctvHistoryFiltered, setHistoryFiltered] = React.useState([]);
  const [currentDateUnit, setCurrentDateUnit] = React.useState('M');
  const miniSearchRef = React.useRef(null);
  const filterRef = React.useRef(null);

  const searchTitle = React.useCallback((title) => {
    const searchPattern =
      hangul.disassemble(title).join('') || MiniSearch.wildcard;
    return miniSearchRef.current.search(searchPattern, SEARCH_OPTION);
  }, []);

  return (
    <Container>
      <Rows>
        {nodesExpanded.map((node) => (
          <RowContainer key={node.id}>
            <CCTV key={node.id}>
              {/* <Action>[{cctv.action}]</Action> */}
              <Title
                id={node.id}
                onClick={()=>{}}
              >
                {node.text}
              </Title>
              <DelButton id={node.id} onClick={removeNode}>
                [Del]
              </DelButton>
            </CCTV>
            <TimeStamp># of leaf: {node.value}</TimeStamp>
          </RowContainer>
        ))}
      </Rows>
      <HistoryCount>
        {nodesExpanded.length} shown
      </HistoryCount>
    </Container>
  )
}

export default React.memo(NodesShown);