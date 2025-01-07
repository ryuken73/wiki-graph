import React from 'react';
import NodeExpanded from './NodeExpanded';
import MiniSearch from 'minisearch';
import hangul from 'hangul-js';
import styled from 'styled-components';
import {
  Container,
  Rows,
  HistoryCount
} from './ContainerStyles';

const Title = styled.div`
  text-align: right;
  color: yellow;
  font-weight: 200;
`
const SEARCH_OPTION = {
  prefix: true,
  fields: ['json.title'],
  fuzzy: 0.3
};

function ExpandedContainer(props) {
  // eslint-disable-next-line react/prop-types
  const {
    nodesExpanded, 
    removeNode, 
    lastNetworkData,
    setLastNetworkData,
    setBacklinksToShow,
    setForwardlinksToShow,
    setActiveExpandedNodeId
  } = props;
  const miniSearchRef = React.useRef(null);

  const searchTitle = React.useCallback((title) => {
    const searchPattern =
      hangul.disassemble(title).join('') || MiniSearch.wildcard;
    return miniSearchRef.current.search(searchPattern, SEARCH_OPTION);
  }, []);

  return (
    <Container>
      <Title>Node Expanded</Title>
      <Rows>
        {nodesExpanded.map((node) => (
          <NodeExpanded
            key={node.id}
            node={node}
            removeNode={removeNode}
            lastNetworkData={lastNetworkData}
            setBacklinksToShow={setBacklinksToShow}
            setForwardlinksToShow={setForwardlinksToShow}
            setLastNetworkData={setLastNetworkData}
            setActiveExpandedNodeId={setActiveExpandedNodeId}
          ></NodeExpanded>
        ))}
      </Rows>
      <HistoryCount>
        {nodesExpanded.length} shown
      </HistoryCount>
    </Container>
  )
}

export default React.memo(ExpandedContainer);