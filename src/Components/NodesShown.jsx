import React from 'react';
import NodeExpanded from './NodeExpanded';
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
  const {
    nodesExpanded, 
    removeNode, 
    setLastNetworkData
  } = props;
  const miniSearchRef = React.useRef(null);

  const searchTitle = React.useCallback((title) => {
    const searchPattern =
      hangul.disassemble(title).join('') || MiniSearch.wildcard;
    return miniSearchRef.current.search(searchPattern, SEARCH_OPTION);
  }, []);

  return (
    <Container>
      <Rows>
        <div>Node Expanded</div>
        {nodesExpanded.map((node) => (
          <NodeExpanded
            key={node.id}
            node={node}
            removeNode={removeNode}
            setLastNetworkData={setLastNetworkData}
          ></NodeExpanded>
        ))}
      </Rows>
      <HistoryCount>
        {nodesExpanded.length} shown
      </HistoryCount>
    </Container>
  )
}

export default React.memo(NodesShown);